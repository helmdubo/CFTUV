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
| не поддержанные фигуры названы исходом, а не обойдены    | `..._named_refusal_...`, `..._holes_2_...` |
| крест ломает сборщик граней, и это нашёл эталон          | `..._cross_breaks_the_face_assembler_...` |

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
    if queue is None:
        assert name == "holes_2", f"{name}: очередь молча пропала при alpha={alpha}"
        return
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


def test_the_holes_2_standard_exists_where_the_face_assembler_refuses():
    """Эталон достаёт дальше очереди, и разрыв назван именем и числом.

    `holes_2` — прямоугольник с двумя квадратными дырами, рёбра соседних дыр
    коллинеарны и одной длины, поэтому ключ участника их не различает и сборщик
    граней отказывает `SUPPORT_LINE_SHARED_BY_SEVERAL_EDGES` ДО всякого счёта.
    Эталон при этом считается и говорит, чего именно очередь не сказала: 188, 384,
    588 против 180, 352, 516 у полос, восемь вогнутых вершин, разность `8*alpha^2`.
    """

    polygon = dict(named_corpus())["holes_2"]
    partition = build_faces(polygon, build_skeleton(polygon))
    assert partition.outcome is FaceOutcome.SUPPORT_LINE_SHARED_BY_SEVERAL_EDGES
    assert partition.faces == ()

    got = []
    for alpha in ("1", "2", "3"):
        standard = _standard(polygon, Fraction(alpha))
        assert standard.reflex_count == 8
        assert standard.strip_deficit == 8 * Fraction(alpha) ** 2
        got.append((standard.mitered_covered, standard.strip_covered))
    assert got == [
        (Fraction(188), Fraction(180)),
        (Fraction(384), Fraction(352)),
        (Fraction(588), Fraction(516)),
    ]


def test_the_cross_breaks_the_face_assembler_and_the_standard_is_what_showed_it():
    """Находка среза: на кресте сборщик граней ломается, и ломается ДВУМЯ способами.

    Крест — прямоугольная фигура с четырьмя вогнутыми вершинами, у которой все
    двенадцать несущих прямых различны (ключ включает `q = |d|^2`, поэтому
    коллинеарные рёбра РАЗНОЙ длины ключом различаются). Тем не менее:

    | крест        | что делает очередь | число |
    |--------------|--------------------|-------|
    | `4 x 4` блок | `WAVEFRONT_LEFT_UNRESOLVED` | граней нет |
    | `5 x 4` блок | `FACE_AREA_DOES_NOT_REPRODUCE_POLYGON` | удвоенный дефект −1 |
    | `6 x 4` блок | `FACE_AREA_DOES_NOT_REPRODUCE_POLYGON` | удвоенный дефект −4 |

    Второй случай БЫЛ тихим: исход говорил `EXACT`, а сумма площадей граней
    меньше площади фигуры на 1/2 либо 2. Ровно поэтому покрытие расходилось с
    эталоном: при alpha = 3 крест `5 x 4` давал 303/2 вместо 152, крест `6 x 4` —
    166 вместо 168, и разность в точности равна дефекту разбиения, а не профилю
    вогнутого угла. Оба поиска split-кандидатов дают один и тот же скелет из 18
    узлов, значит дело в СБОРЩИКЕ, а не в очереди событий.

    Теперь тихого нет: `build_faces` зовёт собственную границу 1 и отказывает
    названным исходом с числом потери в `detail`. Утёкшие числа тест всё равно
    предъявляет — на партиции, которой ВРУЧНУЮ возвращён ярлык `EXACT`. Это не
    обход защиты, а её свидетельство: видно ровно то, что защита остановила.
    """

    square_block = cross(wide=4, tall=4)
    skeleton = build_skeleton(square_block)
    assert skeleton.outcome is SkeletonOutcome.WAVEFRONT_LEFT_UNRESOLVED
    partition = build_faces(square_block, skeleton)
    assert partition.outcome is FaceOutcome.SKELETON_IS_NOT_EXACT
    # Эталон на той же фигуре считается и даёт ответ: 72 / 136 / 136.
    assert [
        _standard(square_block, Fraction(alpha)).mitered_covered
        for alpha in (1, 2, 3)
    ] == [Fraction(72), Fraction(136), Fraction(136)]

    for wide, defect, covered, queue in (
        (5, Fraction(-1), Fraction(152), Fraction(303, 2)),
        (6, Fraction(-4), Fraction(168), Fraction(166)),
    ):
        figure = cross(wide=wide, tall=4)
        partition = build_faces(figure, build_skeleton(figure))
        assert partition.outcome is (
            FaceOutcome.FACE_AREA_DOES_NOT_REPRODUCE_POLYGON
        )
        assert not partition.area_reproduces_polygon
        assert partition.area_defect.as_rational() == defect
        # Причина отказа — число, а не слово: иначе −1 и −4 не различить.
        assert partition.detail == str(defect)
        standard = _standard(figure, Fraction(3))
        assert standard.mitered_covered == covered
        # Ярлык возвращается вручную ровно затем, чтобы предъявить утечку,
        # которую защита теперь останавливает. Через `build_faces` она наружу
        # больше не выходит — это проверяет следующий assert.
        relabelled = replace(partition, outcome=FaceOutcome.EXACT)
        assert (
            coverage_at(relabelled, Fraction(3)).doubled_area.as_rational() / 2
        ) == queue
        # А по-настоящему покрытие отказывает и ничего не считает.
        refused = coverage_at(partition, Fraction(3))
        assert refused.outcome is CoverageOutcome.PARTITION_IS_NOT_EXACT
        assert refused.doubled_area.is_zero
        # Расхождение покрытия — это ровно дефект разбиения, ничего сверх него.
        assert covered - queue == -defect / 2
