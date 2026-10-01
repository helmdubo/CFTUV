"""Слияние граней одной цепи на одной прямой: чистые тесты ядра.

Переехали из `tests/test_envelope_queue_contour_merge.py` вместе с кодом
(`materialize/coalesce.py`): один код слияния на отладочную картинку и на
продуктовый меш, и проверяется он там же, где живёт. Хостовые проверки
(слой скелета, стадия `QUEUE_CONTOUR` на настоящем домене, sidecar) остались в
хостовом файле: они про отображение, а не про слияние.

Вход синтетический и целочисленный, и это не упрощение: слияние решается
ЧЕТЫРЬМЯ точными предикатами (регион, класс несущей прямой, `PhysicalChain`,
семантика владельца), и синтетика позволяет пошевелить каждый из них по
отдельности.
"""

from __future__ import annotations

from fractions import Fraction
from types import SimpleNamespace

from cftuv_envelope.materialize.coalesce import (
    CoveredFaceV1,
    FACE_LOSS_REASONS,
    FaceMatchV1,
    integer_line_class,
    match_region_faces,
    merge_same_chain_faces,
    undirected_span,
)
from cftuv_envelope.wavefront.sqrt_sum import SqrtSumV1


CHAIN = "physical-chain:one"
OTHER_CHAIN = "physical-chain:two"


def _point(x, y):
    return (SqrtSumV1.rational(Fraction(x)), SqrtSumV1.rational(Fraction(y)))


def _face(owner, points, *, chain=CHAIN, spec="", instance=None):
    """Грань стадии контура: контур ПРОТИВ часовой, первое ребро — источник.

    Порядок точек тот же, что объявлен ядром (`build_faces`): `points[0]` и
    `points[1]` — концы ребра домена, дальше узлы скелета.
    """

    contour = tuple(_point(x, y) for x, y in points)
    doubled = SqrtSumV1.zero()
    size = len(contour)
    for index in range(size):
        x0, y0 = contour[index]
        x1, y1 = contour[(index + 1) % size]
        doubled = doubled + (x0 * y1) - (x1 * y0)
    return CoveredFaceV1(
        region_id="region",
        owner=owner,
        envelope_spec_id=spec,
        envelope_instance_id=instance,
        points=contour,
        doubled_area=doubled,
        source_chain_id=chain,
    )


#: Грань левого ребра прямой вершины: источник `(0,0) -> (2,0)`.
def _left(**kwargs):
    return _face((0, 0, 2, 0), ((0, 0), (2, 0), (2, 1), (0, 1)), **kwargs)


#: Грань правого ребра той же прямой: источник `(2,0) -> (4,0)`.
#: Общая граница с левой — отрезок `(2,0) - (2,1)`, та самая дуга из прямой
#: вершины, которая и рисовалась лишним поперечным ребром декали.
def _right(**kwargs):
    return _face((2, 0, 4, 0), ((2, 0), (4, 0), (4, 1), (2, 1)), **kwargs)


#: Грань ВЕРТИКАЛЬНОГО ребра той же цепи: источник `(2,1) -> (2,0)`. Границу с
#: левой делит ту же самую, но прямая у неё ДРУГАЯ — это угол внутри цепи.
def _corner(**kwargs):
    return _face((2, 1, 2, 0), ((2, 1), (2, 0), (4, 0), (4, 1)), **kwargs)


#: Грань скрытой опоры веера: ключ пятиместный, отрезка не задаёт.
def _fan(**kwargs):
    return _face((2, 1, 2, 1, 1), ((2, 1), (2, 0), (4, 0), (4, 1)), **kwargs)


def testinteger_line_class_matches_the_kernel_predicate():
    """Класс прямой считается ТОЙ ЖЕ формулой, что у моста, и без float'ов."""

    from cftuv_envelope.wavefront.bridge import line_class

    for owner in ((0, 0, 2, 0), (2, 0, 4, 0), (2, 1, 2, 0), (0, 0, 3, 5)):
        x0, y0, x1, y1 = owner
        a, b = y0 - y1, x1 - x0
        expected = line_class(
            (Fraction(a), Fraction(b), Fraction(a * x0 + b * y0))
        )
        assert integer_line_class(owner) == expected

    # Ключ веера отрезка не задаёт, вырожденное ребро — тоже.
    assert integer_line_class((2, 1, 2, 1, 1)) is None
    assert integer_line_class((2, 1, 2, 1)) is None
    # Встречные рёбра одной прямой в ОДИН класс не попадают: знак сохранён.
    assert integer_line_class((0, 0, 2, 0)) != integer_line_class((2, 0, 0, 0))


def test_undirected_span_ignores_traversal_direction_but_not_the_fan():
    assert undirected_span((2, 0, 0, 0)) == undirected_span((0, 0, 2, 0))
    assert undirected_span((2, 1, 2, 1, 1)) is None
    assert undirected_span((2, 1, 2, 1)) is None


def test_same_chain_collinear_faces_merge_into_one_contour():
    """Полевой случай: две грани одной цепи на одной прямой — один контур."""

    merged, stats = merge_same_chain_faces((_left(), _right()))

    assert len(merged) == 1
    assert stats.merged_separators == 1
    assert stats.merged_groups == 1
    assert stats.unresolved_groups == 0

    face = merged[0]
    # Владелец назван наименьшим ключом, второй не исчез — он перечислен.
    assert face.owner == (0, 0, 2, 0)
    assert face.merged_owners == ((2, 0, 4, 0),)
    # Площадь слитого контура ТОЧНО равна сумме площадей: 2 + 2 = 4 (удвоенная
    # площадь прямоугольника 4x1). Это же равенство и есть условие принятия.
    assert face.doubled_area.as_rational() == Fraction(8)
    # Разделитель `(2,0) - (2,1)` в контуре не остался ни одним из концов
    # ПОДРЯД: обход идёт по внешней границе объединения.
    corners = [
        (point[0].as_rational(), point[1].as_rational())
        for point in face.points
    ]
    assert len(corners) == 6
    assert set(corners) == {
        (0, 0), (2, 0), (4, 0), (4, 1), (2, 1), (0, 1),
    }
    following = dict(zip(corners, corners[1:] + corners[:1]))
    assert following[(2, 0)] == (4, 0)
    assert following[(2, 1)] == (0, 1)


def test_three_collinear_faces_of_one_chain_report_two_separators():
    """Одна группа, два разделителя. Одним числом эти два случая не различить."""

    far = _face((4, 0, 6, 0), ((4, 0), (6, 0), (6, 1), (4, 1)))
    merged, stats = merge_same_chain_faces((_left(), _right(), far))

    assert len(merged) == 1
    assert stats.merged_groups == 1
    assert stats.merged_separators == 2
    assert merged[0].merged_owners == ((2, 0, 4, 0), (4, 0, 6, 0))
    assert merged[0].doubled_area.as_rational() == Fraction(12)


def test_collinear_faces_of_different_chains_are_not_merged():
    """Негативный контроль: коллинеарность СЛУЧАЙНАЯ, цепи разные."""

    merged, stats = merge_same_chain_faces(
        (_left(), _right(chain=OTHER_CHAIN))
    )

    assert len(merged) == 2
    assert stats == type(stats)(0, 0, 0)
    assert all(face.merged_owners == () for face in merged)


def test_a_corner_inside_one_chain_is_not_merged():
    """Негативный контроль: цепь одна, а прямая другая — это угол цепи."""

    merged, stats = merge_same_chain_faces((_left(), _corner()))

    assert len(merged) == 2
    assert stats.merged_separators == 0
    assert stats.merged_groups == 0
    assert stats.unresolved_groups == 0


def test_a_fan_support_face_is_not_merged():
    """Негативный контроль: у скрытой опоры веера отрезка нет вовсе."""

    merged, stats = merge_same_chain_faces((_left(), _fan()))

    assert len(merged) == 2
    assert stats.merged_separators == 0
    assert stats.merged_groups == 0


def test_faces_of_different_owners_are_not_merged():
    """Негативный контроль: цепь и прямая одни, а огибающие разные."""

    by_spec, stats = merge_same_chain_faces(
        (_left(spec="strip-a"), _right(spec="strip-b"))
    )
    assert len(by_spec) == 2
    assert stats.merged_groups == 0

    # Имя ЭКЗЕМПЛЯРА тоже различает: оно стоит на эффективной alpha, и две
    # разные эффективные alpha — это две огибающие, а не одна.
    by_instance, stats = merge_same_chain_faces(
        (
            _left(spec="strip-a", instance="strip-a@1"),
            _right(spec="strip-a", instance="strip-a@2"),
        )
    )
    assert len(by_instance) == 2
    assert stats.merged_groups == 0


def test_faces_without_a_named_chain_are_not_merged():
    """Цепь не названа — условие (c) не доказано, значит слияния нет."""

    merged, stats = merge_same_chain_faces(
        (_left(chain=None), _right(chain=None))
    )

    assert len(merged) == 2
    assert stats.merged_groups == 0
    assert stats.unresolved_groups == 0


def test_collinear_same_chain_faces_that_do_not_touch_are_not_a_failure():
    """Не смежные грани — не неудача слияния, и кричать о них нельзя.

    Два коллинеарных ребра одной цепи могут стоять по разные стороны домена.
    Сливать там нечего; `CONTOUR_MERGE_BOUNDARY_UNRESOLVED` обязан остаться
    именем настоящей аномалии, а не срабатывать на законном входе.
    """

    apart = _face((10, 0, 12, 0), ((10, 0), (12, 0), (12, 1), (10, 1)))
    merged, stats = merge_same_chain_faces((_left(), apart))

    assert len(merged) == 2
    assert stats.merged_groups == 0
    assert stats.unresolved_groups == 0


def test_a_boundary_that_does_not_prove_itself_stays_unmerged_and_counted():
    """Смежность есть, а обход не сложился: грани остаются, число растёт.

    Вход — аномалия разбиения: два ребра-источника одной цепи претендуют на
    ОДНУ И ТУ ЖЕ грань, то есть одно полуребро приходит дважды в одном
    направлении. Молчаливого второго порядка обхода здесь нет: группа остаётся
    неслитой и названа числом.
    """

    twin = _face((4, 0, 6, 0), ((2, 0), (4, 0), (4, 1), (2, 1)))
    merged, stats = merge_same_chain_faces((_left(), _right(), twin))

    assert len(merged) == 3
    assert stats.merged_groups == 0
    assert stats.merged_separators == 0
    assert stats.unresolved_groups == 1


def test_a_merged_contour_that_is_not_simple_is_refused_by_the_kernel_bound():
    """Обход сложился, а контур перекручен — граница 1 ядра его отвергает.

    Проверяется именно та проверка, которая тождеством НЕ является: площадь на
    перекрученном контуре сходится (сокращение вычитает ноль), а
    трансверсальное самопересечение — нет.
    """

    from cftuv_envelope.wavefront.faces import contour_crossings

    twisted = _face(
        (2, 0, 4, 0), ((2, 0), (4, 0), (4, 2), (3, -1), (2, 1))
    )
    merged, stats = merge_same_chain_faces((_left(), twisted))

    assert stats.unresolved_groups == 1
    assert stats.merged_groups == 0
    assert len(merged) == 2
    # Перекрут лежит во ВХОДЕ, и это доказано тем же предикатом ядра: без него
    # тест мерил бы отказ, не назвав его причины.
    assert contour_crossings(twisted.points)


# --------------------------------------------------------------------------
# Судьба КАЖДОЙ грани покрытия: счёт вместо молчаливого пропуска
# --------------------------------------------------------------------------


def _named(owner, area=2):
    """Грань покрытия: владелец и ТОЧНАЯ удвоенная площадь."""

    return SimpleNamespace(
        region_id="region",
        owner=owner,
        envelope_spec_id="spec",
        envelope_instance_id="instance",
        doubled_area=SqrtSumV1.rational(Fraction(area)),
    )


def _contour(owner, points):
    return SimpleNamespace(owner=owner, points=tuple(_point(x, y) for x, y in points))


SQUARE = ((0, 0), (2, 0), (2, 1), (0, 1))
LEFT = (0, 0, 2, 0)
RIGHT = (2, 0, 4, 0)


def _legacy_covered_faces(covered, contours, spans):
    """Прежний цикл ОТЛАДКИ слово в слово: оракул того, что хост рисует.

    Это не проверяемый код, а его замороженная копия до правки: отладочный
    путь обязан отдавать ТЕ ЖЕ грани, а счёт потерь — добавление рядом.
    """

    faces = []
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


def test_a_clean_region_balances_and_loses_nothing():
    covered = SimpleNamespace(faces=(_named(LEFT), _named(RIGHT)))
    contours = (_contour(LEFT, SQUARE), _contour(RIGHT, SQUARE))
    faces, match = match_region_faces(covered, contours, {})

    assert [item.owner for item in faces] == [LEFT, RIGHT]
    assert (match.faces_in, match.matched, match.empty_after_clip) == (2, 2, 0)
    assert match.lost_total == 0 and match.balanced
    # Ноль потери — измерение: все причины названы и равны нулю.
    assert dict(match.lost) == {name: 0 for name in FACE_LOSS_REASONS}
    assert match.loss_counters() == ()


def test_a_face_behind_the_front_is_empty_not_lost():
    """Контур короче трёх точек И площадь ровно ноль — законная судьба."""

    covered = SimpleNamespace(faces=(_named(LEFT), _named(RIGHT, area=0)))
    contours = (_contour(LEFT, SQUARE), _contour(RIGHT, ()))
    faces, match = match_region_faces(covered, contours, {})

    assert [item.owner for item in faces] == [LEFT]
    assert match.empty_after_clip == 1 and match.lost_total == 0
    assert match.balanced
    assert match.loss_counters() == ()


def test_every_way_to_lose_a_face_is_counted_under_its_own_name():
    # Хвост без контуров, разошедшийся владелец, короткий контур с площадью.
    covered = SimpleNamespace(
        faces=(_named(LEFT), _named(RIGHT), _named((4, 0, 6, 0)), _named((6, 0, 8, 0)))
    )
    contours = (
        _contour(LEFT, SQUARE),
        _contour((9, 9, 9, 9), SQUARE),
        _contour((4, 0, 6, 0), ((4, 0), (6, 0))),
    )
    faces, match = match_region_faces(covered, contours, {})

    assert [item.owner for item in faces] == [LEFT]
    assert dict(match.lost) == {
        "CONTOUR_MISSING": 1,
        "OWNER_MISMATCH": 1,
        "SHORT_CONTOUR_WITH_AREA": 1,
    }
    assert match.faces_in == 4 and match.lost_faces == 3 and match.balanced
    assert {name for name, _owner in match.lost_owners} == set(FACE_LOSS_REASONS)
    counters = dict(match.counters())
    assert counters["MATERIALIZE_FACES_IN"] == 4
    assert counters["MATERIALIZE_FACES_LOST"] == 3
    assert counters["MATERIALIZE_FACES_LOST_OWNER_MISMATCH"] == 1
    # Для отладки потеря называется ЧИСЛОМ, и только когда она есть.
    assert dict(match.loss_counters())["MATERIALIZE_FACES_LOST"] == 3


def test_a_surplus_contour_is_named_but_is_not_a_coverage_face():
    covered = SimpleNamespace(faces=(_named(LEFT),))
    contours = (_contour(LEFT, SQUARE), _contour(RIGHT, SQUARE), _contour(RIGHT, ()))
    _faces, match = match_region_faces(covered, contours, {})

    assert match.surplus_contours == 1
    assert match.lost_faces == 0 and match.lost_total == 1
    # Лишний контур не часть баланса `faces_in`: это не грань покрытия.
    assert match.balanced
    assert dict(match.counters())["MATERIALIZE_CONTOURS_WITHOUT_FACE"] == 1


def test_the_debug_faces_are_exactly_the_legacy_loop_with_or_without_losses():
    """Отладочный путь не изменился ни на грань: сравнение с замороженным циклом."""

    spans = {undirected_span(LEFT): "chain:one"}
    cases = (
        (
            SimpleNamespace(faces=(_named(LEFT), _named(RIGHT))),
            (_contour(LEFT, SQUARE), _contour(RIGHT, SQUARE)),
        ),
        (
            SimpleNamespace(faces=(_named(LEFT), _named(RIGHT, area=0))),
            (_contour(LEFT, SQUARE), _contour(RIGHT, ())),
        ),
        (
            SimpleNamespace(faces=(_named(LEFT), _named(RIGHT), _named((4, 0, 6, 0)))),
            (_contour(LEFT, SQUARE), _contour((9, 9, 9, 9), SQUARE)),
        ),
        (SimpleNamespace(faces=(_named(LEFT),)), ()),
        (SimpleNamespace(faces=()), (_contour(LEFT, SQUARE),)),
    )
    for covered, contours in cases:
        faces, match = match_region_faces(covered, contours, spans)
        assert faces == _legacy_covered_faces(covered, contours, spans)
        assert match.balanced


def test_match_sums_are_elementwise_and_keep_the_balance():
    left = FaceMatchV1(faces_in=3, matched=2, empty_after_clip=1)
    right = FaceMatchV1(
        faces_in=2,
        matched=1,
        lost=(
            ("CONTOUR_MISSING", 1),
            ("OWNER_MISMATCH", 0),
            ("SHORT_CONTOUR_WITH_AREA", 0),
        ),
        lost_owners=(("CONTOUR_MISSING", (1, 2, 3, 4)),),
    )
    total = left + right
    assert (total.faces_in, total.matched, total.empty_after_clip) == (5, 3, 1)
    assert total.lost_faces == 1 and total.balanced
    assert "CONTOUR_MISSING=1" in total.describe_losses()
    assert "(1, 2, 3, 4)" in total.describe_losses()
