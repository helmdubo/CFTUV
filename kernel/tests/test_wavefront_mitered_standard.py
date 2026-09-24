"""Покрытие против НЕЗАВИСИМОГО эталона: первая такая проверка в проекте.

Все прежние проверки покрытия замкнуты на сам код покрытия.
`interactions/policy_b.py:969-976` сравнивает результат кроя с `RawCoverage`, то
есть покрытие с самим собой до кроя; `test_wavefront_coverage.py` проверяет
границы, которые `coverage.py` объявила о себе сама, на её же результате. Ни то
ни другое не отвечает на вопрос «а верно ли само покрываемое множество», и
поэтому дыра `alpha^2` у вогнутой вершины дожила до этого среза.

Здесь эталон посчитан ВНЕ обоих путей — `mitered_standard.py`, точные дроби,
замкнутая форма через вписанные прямоугольники, — и им проверяются оба пути:

| что проверяется                                        | тест |
|--------------------------------------------------------|------|
| эталон воспроизводит замкнутую форму `ell` 44/63/80/108 | `..._reproduces_the_closed_form_...` |
| очередь совпадает с эталоном ТОЧНО                      | `..._queue_coverage_equals_...` |
| полосы расходятся, и разность названа числом            | `..._strips_fall_short_...` |
| разность = сумме `alpha^2`, когда квадраты не пересеклись| `..._deficit_is_the_whole_budget_...` |
| разность МЕНЬШЕ суммы, и обе причины разделены          | `..._deficit_falls_below_...` |
| модель полос — это и есть `RawCoverage` конвейера        | `..._strip_model_is_the_raw_coverage_...` |
| не поддержанные фигуры названы исходом, а не обойдены    | `..._named_refusal_...` |
| `holes_2` дошёл до эталона, которого раньше не достигал  | `..._holes_2_queue_now_reaches_...` |
| крест сходится с эталоном на всей сетке 3..9             | `..._cross_now_agrees_with_...` |

Пропущенных строк в сверке очереди с эталоном НЕТ НИ ОДНОЙ. Оговорка про
`holes_2` («очередь до счёта не дошла») снята срезом, сменившим ключ участника
с несущей прямой на вхождение ребра.

Точка семейства названа явно и здесь тоже: эталон МИТРОВАННЫЙ. Круглый отступ
(Минковский, «эрозия») даёт третье число `43 + pi/4` и эталоном не является;
`alpha^2` — верхняя граница семейства профилей вогнутого угла, а не допуск.
"""

from __future__ import annotations

from dataclasses import replace
from fractions import Fraction

import pytest

import cftuv_envelope as kernel
from cftuv_envelope.reference.planar_types import ExactScalar, exact_sign
from cftuv_envelope.wavefront import build_skeleton
from cftuv_envelope.wavefront.coverage import CoverageOutcome, coverage_at
from cftuv_envelope.wavefront.faces import FaceOutcome, build_faces
from cftuv_envelope.wavefront.skeleton import SkeletonOutcome

from mitered_standard import (
    StandardOutcome,
    axis_edges,
    mitered_inset_rectangles,
    rectilinear_standard,
)
from reference_factories import straight_snapshot
from wavefront_cases import cross, named_corpus


# Прямоугольное семейство корпуса: только у него митрованный отступ считается в
# рациональных числах. `comb`, `right_triangle`, `diamond`, `star`, `field` сюда
# не входят, и это проверяется отдельным тестом, а не молчанием.
RECTILINEAR = tuple(
    (name, polygon)
    for name, polygon in named_corpus()
    if axis_edges(polygon) is not None
)
RECTILINEAR_IDS = tuple(name for name, _ in RECTILINEAR)


# Лестница alpha на каждую фигуру. Ступени подобраны так, чтобы среди них были и
# те, где ядро ещё не выродилось, и те, где выродилось (покрытие = вся фигура):
# вырождение — законный случай, а не повод для отказа.
LADDER = {
    "axis_square": ("1", "2", "4", "5"),
    "axis_rectangle": ("1", "2", "4", "5"),
    "ell": ("1", "3/2", "2", "3"),
    "hole_1": ("1", "2", "3", "6"),
    "holes_2": ("1", "2", "3"),
    # Крест вошёл в корпус этим срезом. Ступени 3 и 4 взяты не для полноты: при
    # alpha = 3 квадраты `alpha^2` четырёх вогнутых вершин ПЕРЕСЕКАЮТСЯ (24
    # против бюджета 36), а при alpha = 4 покрытие уже вся фигура.
    "cross": ("1", "2", "3", "4"),
    "staircase": ("1", "2", "4", "5"),
    "u_shape": ("1", "2", "3", "5"),
    "double_notch": ("1", "3/2", "2", "3", "4"),
}


# Числа среза: `(эталон, полосы, разность, объединение квадратов, сумма alpha^2)`.
# Таблица записана целиком, а не выведена по ходу, потому что тест обязан падать
# при СМЕНЕ числа, а не подстраиваться под неё.
EXPECTED = {
    ("axis_square", "1"): ("28", "28", "0", "0", "0"),
    ("axis_square", "2"): ("48", "48", "0", "0", "0"),
    ("axis_square", "4"): ("64", "64", "0", "0", "0"),
    ("axis_square", "5"): ("64", "64", "0", "0", "0"),
    ("axis_rectangle", "1"): ("52", "52", "0", "0", "0"),
    ("axis_rectangle", "2"): ("96", "96", "0", "0", "0"),
    ("axis_rectangle", "4"): ("160", "160", "0", "0", "0"),
    ("axis_rectangle", "5"): ("160", "160", "0", "0", "0"),
    ("ell", "1"): ("44", "43", "1", "1", "1"),
    ("ell", "3/2"): ("63", "243/4", "9/4", "9/4", "9/4"),
    ("ell", "2"): ("80", "76", "4", "4", "4"),
    ("ell", "3"): ("108", "99", "9", "9", "9"),
    ("hole_1", "1"): ("112", "108", "4", "4", "4"),
    ("hole_1", "2"): ("224", "208", "16", "16", "16"),
    ("hole_1", "3"): ("336", "300", "36", "36", "36"),
    ("hole_1", "6"): ("336", "336", "0", "144", "144"),
    ("holes_2", "1"): ("188", "180", "8", "8", "8"),
    ("holes_2", "2"): ("384", "352", "32", "32", "32"),
    ("holes_2", "3"): ("588", "516", "72", "72", "72"),
    ("cross", "1"): ("72", "68", "4", "4", "4"),
    ("cross", "2"): ("136", "120", "16", "16", "16"),
    ("cross", "3"): ("168", "144", "24", "24", "36"),
    ("cross", "4"): ("168", "144", "24", "24", "64"),
    ("staircase", "1"): ("44", "42", "2", "2", "2"),
    ("staircase", "2"): ("80", "72", "8", "8", "8"),
    ("staircase", "4"): ("96", "96", "0", "32", "32"),
    ("staircase", "5"): ("96", "96", "0", "39", "50"),
    ("u_shape", "1"): ("60", "58", "2", "2", "2"),
    ("u_shape", "2"): ("112", "104", "8", "8", "8"),
    ("u_shape", "3"): ("135", "127", "8", "18", "18"),
    ("u_shape", "5"): ("135", "135", "0", "50", "50"),
    ("double_notch", "1"): ("76", "72", "4", "4", "4"),
    ("double_notch", "3/2"): ("106", "197/2", "15/2", "9", "9"),
    ("double_notch", "2"): ("129", "119", "10", "16", "16"),
    ("double_notch", "3"): ("151", "145", "6", "33", "36"),
    ("double_notch", "4"): ("151", "151", "0", "44", "64"),
}


def _rows():
    for name, polygon in RECTILINEAR:
        for alpha in LADDER[name]:
            yield name, polygon, Fraction(alpha)


ROWS = tuple(_rows())
ROW_IDS = tuple(f"{name}-{alpha}" for name, _, alpha in ROWS)


def _standard(polygon, alpha: Fraction):
    standard = rectilinear_standard(polygon, alpha)
    assert standard.outcome is StandardOutcome.EXACT, standard.detail
    return standard


def _queue_area(polygon, alpha: Fraction) -> Fraction | None:
    """Площадь покрытия по очереди, либо `None`, если очередь до неё не дошла.

    `None` возвращается ТОЛЬКО тогда, когда сборщик граней сам отказал именованным
    исходом либо нарушил собственную границу 1. Отличать эти два случая от нуля
    обязательно: ноль — это ответ, а отказ — не ответ.
    """

    partition = build_faces(polygon, build_skeleton(polygon))
    if partition.outcome is not FaceOutcome.EXACT:
        return None
    if not partition.area_reproduces_polygon:
        return None
    coverage = coverage_at(partition, alpha)
    assert coverage.outcome is CoverageOutcome.EXACT
    return coverage.doubled_area.as_rational() / 2


# --------------------------------------------------------------------------
# 1. Эталон отвечает за себя сам
# --------------------------------------------------------------------------


def test_the_mitered_standard_reproduces_the_closed_form_of_the_ell():
    """Замкнутая форма из карты среза, посчитанная эталоном: 44 / 63 / 80 / 108.

    Это те самые числа, которыми митрованный ответ был проверен независимо от
    обеих реализаций, и колонка полос 43 / 243/4 / 76 / 99 — ровно на `alpha^2`
    ниже. Совпадение здесь важнее любого другого в файле: если эталон не даст их,
    все остальные сравнения станут сравнениями с неизвестно чем.
    """

    polygon = dict(named_corpus())["ell"]
    mitered = {}
    strips = {}
    for alpha in ("1", "3/2", "2", "3"):
        standard = _standard(polygon, Fraction(alpha))
        mitered[alpha] = standard.mitered_covered
        strips[alpha] = standard.strip_covered
    assert mitered == {
        "1": Fraction(44),
        "3/2": Fraction(63),
        "2": Fraction(80),
        "3": Fraction(108),
    }
    assert strips == {
        "1": Fraction(43),
        "3/2": Fraction(243, 4),
        "2": Fraction(76),
        "3": Fraction(99),
    }


def test_the_inset_of_the_ell_is_literally_the_two_rectangles_of_the_formula():
    """`108 - |[a,12-a]x[a,6-a] U [a,6-a]x[a,12-a]|` — не пересказ, а конструкция.

    Отступ строится объединением вписанных прямоугольников, сжатых на alpha, и у
    `ell` их ровно два. Тест смотрит на сами прямоугольники, а не только на
    площадь: совпавшая площадь при других прямоугольниках означала бы, что сошлось
    случайно.
    """

    polygon = dict(named_corpus())["ell"]
    alpha = Fraction(1)
    assert set(mitered_inset_rectangles(polygon, alpha)) == {
        (Fraction(1), Fraction(11), Fraction(1), Fraction(5)),
        (Fraction(1), Fraction(5), Fraction(1), Fraction(11)),
    }


def test_a_slanted_edge_is_a_named_refusal_of_the_standard_not_a_wrong_number():
    """Эталон живёт только на осевых фигурах и говорит это ИСХОДОМ.

    `comb` скошен зубьями, `right_triangle` и `diamond` — гипотенузой, `star` и
    полевой контур — общим положением. Ни на одной из них митрованный отступ не
    прямоугольный, и подсунуть вместо него что-нибудь похожее нельзя.
    """

    refused = []
    for name, polygon in named_corpus():
        standard = rectilinear_standard(polygon, Fraction(1))
        if standard.outcome is not StandardOutcome.EXACT:
            refused.append((name, standard.outcome))
    assert refused
    assert all(
        outcome is StandardOutcome.EDGE_IS_NOT_AXIS_PARALLEL
        for _, outcome in refused
    )
    assert {name for name, _ in refused} == {
        name
        for name, polygon in named_corpus()
        if axis_edges(polygon) is None
    }
    assert "comb_2" in {name for name, _ in refused}


def test_a_negative_alpha_is_a_named_outcome_of_the_standard_too():
    """Отрицательная alpha — не пустое покрытие, а названный отказ."""

    polygon = dict(named_corpus())["ell"]
    standard = rectilinear_standard(polygon, Fraction(-1))
    assert standard.outcome is StandardOutcome.ALPHA_IS_NEGATIVE
    assert standard.mitered_covered == 0
    assert standard.polygon_area == 108


# --------------------------------------------------------------------------
# 2. Очередь против эталона: главная проверка среза
# --------------------------------------------------------------------------


@pytest.mark.parametrize("name,polygon,alpha", ROWS, ids=ROW_IDS)
def test_the_queue_coverage_equals_the_independent_mitered_standard_exactly(
    name, polygon, alpha
):
    """Очередь обязана дать ТО ЖЕ множество, что и эталон, посчитанный вне её.

    Совпадение точное, в рациональных числах: у прямоугольной фигуры площадь
    покрытия рациональна, поэтому `as_rational()` сравнимо с `Fraction` побитово,
    без единого порога. Там, где сборщик граней отказал именованным исходом,
    сравнивать нечего, и это НЕ засчитывается за совпадение.
    """

    standard = _standard(polygon, alpha)
    queue = _queue_area(polygon, alpha)
    # Пропусков больше НЕТ ни одного. Раньше здесь стояла оговорка про `holes_2`:
    # сборщик граней отказывал на нём, потому что ключом участника была несущая
    # прямая, а у рёбер соседних дыр она общая. Ключ стал вхождением ребра, и
    # три строки `holes_2` перешли из «сравнивать нечем» в «совпало точно».
    assert queue is not None, f"{name}: очередь не дошла при alpha={alpha}"
    assert queue == standard.mitered_covered, (
        f"{name} при alpha={alpha}: очередь {queue}, эталон "
        f"{standard.mitered_covered}, разность {queue - standard.mitered_covered}"
    )


@pytest.mark.parametrize("name,polygon,alpha", ROWS, ids=ROW_IDS)
def test_the_standard_holds_the_numbers_of_the_slice_for_every_figure(
    name, polygon, alpha
):
    """Вся таблица среза целиком: эталон, полосы, разность, объединение, бюджет."""

    standard = _standard(polygon, alpha)
    expected = EXPECTED[(name, str(alpha))]
    assert (
        str(standard.mitered_covered),
        str(standard.strip_covered),
        str(standard.strip_deficit),
        str(standard.reflex_square_union),
        str(standard.reflex_square_budget),
    ) == expected


@pytest.mark.parametrize("name,polygon,alpha", ROWS, ids=ROW_IDS)
def test_the_strips_never_cover_more_than_the_mitered_standard(
    name, polygon, alpha
):
    """Порядок трёх ответов не случаен: полосы <= митрованный <= вся фигура.

    Полосы неверны при ЛЮБОМ профиле вогнутого угла — пустой квадрат не даёт ни
    один из них, — поэтому неравенство одностороннее, и обратного случая быть не
    может ни на одной фигуре и ни при какой alpha.
    """

    standard = _standard(polygon, alpha)
    assert standard.strip_covered <= standard.mitered_covered
    assert standard.mitered_covered <= standard.polygon_area
    assert standard.strip_deficit >= 0
    assert standard.strip_deficit <= standard.reflex_square_union
    assert standard.reflex_square_union <= standard.reflex_square_budget


def test_a_convex_figure_leaves_the_two_paths_nothing_to_disagree_about():
    """Отрицательный контроль: без вогнутой вершины разность равна НУЛЮ.

    Без него «эталон больше полос» свелось бы к «эталон просто другой».
    """

    for name in ("axis_square", "axis_rectangle"):
        polygon = dict(named_corpus())[name]
        for alpha in LADDER[name]:
            standard = _standard(polygon, Fraction(alpha))
            assert standard.reflex_count == 0
            assert standard.strip_deficit == 0
            assert standard.strip_covered == standard.mitered_covered


# --------------------------------------------------------------------------
# 3. Разность полос: где она равна сумме alpha^2, а где меньше и почему
# --------------------------------------------------------------------------


def test_the_deficit_is_the_whole_budget_where_the_reflex_squares_are_disjoint():
    """Разность = сумме `alpha^2` по вогнутым вершинам: 15 строк из 24 вогнутых.

    Это и есть измеренное подтверждение того, что теряют именно квадраты вогнутых
    вершин, а не что-то ещё: у `ell` вершина одна и разность `alpha^2`, у
    `staircase` и `u_shape` две и разность `2*alpha^2`, у `hole_1` четыре и
    `4*alpha^2`, у `holes_2` восемь и `8*alpha^2`.
    """

    full = []
    for name, polygon, alpha in ROWS:
        standard = _standard(polygon, alpha)
        if standard.reflex_count and standard.deficit_is_the_full_budget:
            full.append((name, str(alpha), str(standard.strip_deficit)))
    assert full == [
        ("ell", "1", "1"),
        ("ell", "3/2", "9/4"),
        ("ell", "2", "4"),
        ("ell", "3", "9"),
        ("hole_1", "1", "4"),
        ("hole_1", "2", "16"),
        ("hole_1", "3", "36"),
        ("holes_2", "1", "8"),
        ("holes_2", "2", "32"),
        ("holes_2", "3", "72"),
        ("cross", "1", "4"),
        ("cross", "2", "16"),
        ("staircase", "1", "2"),
        ("staircase", "2", "8"),
        ("u_shape", "1", "2"),
        ("u_shape", "2", "8"),
        ("double_notch", "1", "4"),
    ]
    # И на каждой из них разность — ровно `reflex_count * alpha^2`.
    for name, alpha, deficit in full:
        polygon = dict(named_corpus())[name]
        standard = _standard(polygon, Fraction(alpha))
        assert Fraction(deficit) == (
            standard.reflex_count * Fraction(alpha) ** 2
        )


def test_the_deficit_falls_below_the_budget_when_the_reflex_squares_intersect():
    """Отрицательный контроль: при большой alpha квадраты ОБЯЗАНЫ пересечься.

    Тогда разность перестаёт быть суммой `alpha^2`, и это не дефект. Числа:

    | фигура         | alpha | объединение квадратов | сумма `alpha^2` | недобор |
    |----------------|------:|----------------------:|----------------:|--------:|
    | `staircase`    |     5 |                    39 |              50 |      11 |
    | `double_notch` |     3 |                    33 |              36 |       3 |
    | `double_notch` |     4 |                    44 |              64 |      20 |

    Самый чистый случай — крест: у него четыре квадрата сидят в углах блока
    `4 x 4`, и при alpha = 3 их объединение равно ровно площади блока, 16, при
    сумме 36. Недобор 20 — это только пересечение, потому что разность там равна
    объединению.
    """

    measured = []
    for name, polygon, alpha in ROWS:
        standard = _standard(polygon, alpha)
        if standard.reflex_square_union < standard.reflex_square_budget:
            measured.append(
                (
                    name,
                    str(alpha),
                    str(standard.reflex_square_union),
                    str(standard.reflex_square_budget),
                )
            )
    assert measured == [
        ("cross", "3", "24", "36"),
        ("cross", "4", "24", "64"),
        ("staircase", "5", "39", "50"),
        ("double_notch", "3", "33", "36"),
        ("double_notch", "4", "44", "64"),
    ]

    square_block = cross(wide=4, tall=4)
    standard = _standard(square_block, Fraction(3))
    assert standard.reflex_count == 4
    assert standard.reflex_square_union == 16
    assert standard.reflex_square_budget == 36
    # Разность равна объединению: до квадратов креста чужая юбка не дошла,
    # весь недобор до бюджета — чистое пересечение квадратов.
    assert standard.strip_deficit == 16


def test_the_deficit_falls_below_the_union_when_a_foreign_strip_reaches_it():
    """Вторая причина «меньше суммы», и она другая: до квадрата дошла ЧУЖАЯ юбка.

    Разделять их обязательно, иначе «разность меньше» означало бы что угодно:

    | фигура         | alpha | разность | объединение квадратов | съела чужая юбка |
    |----------------|------:|---------:|----------------------:|-----------------:|
    | `hole_1`       |     6 |        0 |                   144 |              144 |
    | `staircase`    |     4 |        0 |                    32 |               32 |
    | `staircase`    |     5 |        0 |                    39 |               39 |
    | `u_shape`      |     3 |        8 |                    18 |               10 |
    | `u_shape`      |     5 |        0 |                    50 |               50 |
    | `double_notch` |   3/2 |     15/2 |                     9 |              3/2 |
    | `double_notch` |     2 |       10 |                    16 |                6 |
    | `double_notch` |     3 |        6 |                    33 |               27 |
    | `double_notch` |     4 |        0 |                    44 |               44 |
    """

    measured = []
    for name, polygon, alpha in ROWS:
        standard = _standard(polygon, alpha)
        if standard.strip_deficit < standard.reflex_square_union:
            measured.append(
                (
                    name,
                    str(alpha),
                    str(standard.strip_deficit),
                    str(standard.reflex_square_union),
                    str(standard.reflex_square_union - standard.strip_deficit),
                )
            )
    assert measured == [
        ("hole_1", "6", "0", "144", "144"),
        ("staircase", "4", "0", "32", "32"),
        ("staircase", "5", "0", "39", "39"),
        ("u_shape", "3", "8", "18", "10"),
        ("u_shape", "5", "0", "50", "50"),
        ("double_notch", "3/2", "15/2", "9", "3/2"),
        ("double_notch", "2", "10", "16", "6"),
        ("double_notch", "3", "6", "33", "27"),
        ("double_notch", "4", "0", "44", "44"),
    ]


# --------------------------------------------------------------------------
# 4. Модель полос — это и есть RawCoverage, а не похожая на него величина
# --------------------------------------------------------------------------


def _raw_coverage_area(polygon, alpha: str):
    """`RawCoverage` конвейера на том же входе, что видит очередь.

    Источником служит КАЖДОЕ ребро контура с единичной скоростью — ровно та
    подстановка, которой мерился попарный путь в Q3.
    """

    face = tuple((float(x), float(y)) for x, y in polygon.outer.points)
    routes = tuple(
        {
            "name": f"e{index}",
            "points": (face[index], face[(index + 1) % len(face)]),
        }
        for index in range(len(face))
    )
    snapshot, request = straight_snapshot(
        faces=(face,),
        source_routes=routes,
        alpha=alpha,
        revision_name=f"mitered-standard-{len(face)}-{alpha}",
    )
    compiled = kernel.compile_reference_envelopes(snapshot, request)
    assert compiled.compilation is not None, compiled.diagnostics
    raw = kernel.evaluate_reference_raw_coverage(
        compiled.compilation, request.requested_alpha
    ).raw_coverage
    assert raw is not None
    return ExactScalar(raw.exact_area_expression).as_expr()


RAW_CASES = (
    ("axis_square", "1", 28),
    ("axis_square", "2", 48),
    ("axis_rectangle", "1", 52),
    ("axis_rectangle", "2", 96),
    ("ell", "1", 43),
    ("ell", "2", 76),
    ("staircase", "1", 42),
    ("staircase", "2", 72),
    ("u_shape", "1", 58),
    ("u_shape", "2", 104),
    ("double_notch", "1", 72),
    ("double_notch", "2", 119),
)


@pytest.mark.parametrize(
    "name,alpha,area", RAW_CASES, ids=[f"{n}-{a}" for n, a, _ in RAW_CASES]
)
def test_the_strip_model_is_the_raw_coverage_of_the_pairwise_path(
    name, alpha, area
):
    """Полосы эталона — не «похожая модель», а тот же ответ, что даёт конвейер.

    Без этого теста разность эталона и полос была бы разностью с выдумкой. Здесь
    же на двенадцати входах модель полос совпадает с `RawCoverage` ТОЧНО, тем же
    предикатом `exact_sign`, которым крой проверяет себя на `policy_b.py:971`.
    """

    polygon = dict(named_corpus())[name]
    standard = _standard(polygon, Fraction(alpha))
    raw = _raw_coverage_area(polygon, alpha)
    assert exact_sign(raw - area) == 0
    assert standard.strip_covered == area


# --------------------------------------------------------------------------
# 5. Где эталон применим, а очередь нет: назвать, а не обойти
# --------------------------------------------------------------------------


def test_the_holes_2_queue_now_reaches_the_standard_that_used_to_outrun_it():
    """`holes_2` перешёл из «эталон достаёт дальше» в «оба дают одно и то же».

    Было: рёбра соседних дыр коллинеарны и ОДНОЙ длины, ключом участника
    служила несущая прямая `(a, b, c, q)`, и сборщик отказывал
    `SUPPORT_LINE_SHARED_BY_SEVERAL_EDGES` ДО всякого счёта. Эталон при этом
    считался и говорил, чего очередь не сказала: 188, 384, 588 против 180, 352,
    516 у полос.

    Стало: ключ участника — вхождение ребра, грани собираются, и очередь даёт
    ровно те же 188, 384, 588. Это и есть независимая проверка починки: эталон
    посчитан в `mitered_standard.py` точными дробями через вписанные
    прямоугольники и про сборщик граней не знает ничего.
    """

    polygon = dict(named_corpus())["holes_2"]
    partition = build_faces(polygon, build_skeleton(polygon))
    assert partition.outcome is FaceOutcome.EXACT, partition.detail
    assert len(partition.faces) == 12
    assert partition.area_defect.terms == ()

    got = []
    for alpha in ("1", "2", "3"):
        standard = _standard(polygon, Fraction(alpha))
        assert standard.reflex_count == 8
        assert standard.strip_deficit == 8 * Fraction(alpha) ** 2
        queue = _queue_area(polygon, Fraction(alpha))
        assert queue == standard.mitered_covered, alpha
        got.append((standard.mitered_covered, standard.strip_covered, queue))
    assert got == [
        (Fraction(188), Fraction(180), Fraction(188)),
        (Fraction(384), Fraction(352), Fraction(384)),
        (Fraction(588), Fraction(516), Fraction(588)),
    ]


def test_the_cross_now_agrees_with_the_independent_standard_on_the_whole_grid():
    """Крест был находкой среза и стал его ПРОВЕРКОЙ, и проверка независимая.

    Эталон посчитан вне обоих путей — точными дробями, замкнутой формой через
    вписанные прямоугольники. Он не знает ни про очередь событий, ни про
    сборщик граней, поэтому совпадение с ним не может быть следствием общей
    ошибки. Именно он показал, что на кресте сборщик ломается.

    История, ради которой запись остаётся видимой:

    | крест        | было | стало |
    |--------------|------|-------|
    | `4 x 4` блок | `WAVEFRONT_LEFT_UNRESOLVED`, граней нет | `EXACT`, покрытие 72 / 136 / 136 |
    | `5 x 4` блок | `FACE_AREA_DOES_NOT_REPRODUCE_POLYGON`, дефект −1, покрытие 303/2 вместо 152 | `EXACT`, 152 |
    | `6 x 4` блок | то же, дефект −4, покрытие 166 вместо 168 | `EXACT`, 168 |

    Проверяется не три строки, а вся сетка `wide, tall` из 3..9 на четырёх
    alpha: 49 * 4 = 196 сравнений с эталоном, и все точные.
    """

    assert [
        _standard(cross(wide=4, tall=4), Fraction(alpha)).mitered_covered
        for alpha in (1, 2, 3)
    ] == [Fraction(72), Fraction(136), Fraction(136)]

    compared = 0
    for wide in range(3, 10):
        for tall in range(3, 10):
            figure = cross(wide=wide, tall=tall)
            partition = build_faces(figure, build_skeleton(figure))
            assert partition.outcome is FaceOutcome.EXACT, (wide, tall)
            assert partition.area_reproduces_polygon, (wide, tall)
            for alpha in (Fraction(0), Fraction(1), Fraction(2), Fraction(3)):
                queue = _queue_area(figure, alpha)
                standard = _standard(figure, alpha)
                assert queue == standard.mitered_covered, (wide, tall, alpha)
                compared += 1
    assert compared == 49 * 4
