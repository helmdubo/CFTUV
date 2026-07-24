"""Угловая мера: оболочка обязана совпадать с точным путём и накрывать истину.

Блокер, ради которого написан `reference/angle_measure.py`: доля π обязана была
быть точной рациональной дробью, иначе патч отвергался целиком
(`EXACT_ANGULAR_CERTIFICATE_UNAVAILABLE`). Полевые данные `building.002`
показали, что валят не экзотические углы, а шум координат на прямых: меш строго
осевой на вид, но binary64 даёт 89.9996° с долей 0.4999980324.

Здесь проверяется ровно то, что делает замену законной:

* точный путь отвечает теми же числами, что и раньше — дайджест закреплён;
* там, где отвечают оба пути, оболочка накрывает точную дробь;
* оболочка накрывает истинное значение и снаружи, и изнутри — она сертификат,
  а не оценка;
* полевые углы принимаются и разрешают профиль;
* то, что оболочка не разделила, даёт именованный отказ, а не молчание.
"""

from __future__ import annotations

from decimal import Decimal
from fractions import Fraction
from hashlib import sha256
import math

import pytest
import sympy as sp
from mpmath import iv

from cftuv_envelope.codec import canonical_json_bytes
from cftuv_envelope.contracts.analysis import (
    AngleNormalizationLaw,
    CertifiedReflexAngleMeasureV1,
    TurnOrientation,
)
from cftuv_envelope.numeric import IntervalEndpointKind
from cftuv_envelope.reference import angle_measure
from cftuv_envelope.reference.compile import _resolve_hidden_edge_count
from cftuv_envelope.validation import (
    _interval_strictly_above,
    _interval_strictly_below,
)


# Точная доля π -> (sin, cos) с тем же atan2. Пары не нормированы намеренно:
# atan2 инвариантен к положительному масштабу, а метрика ядра подаёт сюда
# именно ненормированные выражения с радикалами.
EXACT_CORNERS = {
    Fraction(1, 2): (sp.Integer(1), sp.Integer(0)),
    Fraction(1, 4): (sp.sqrt(2) / 2, sp.sqrt(2) / 2),
    Fraction(3, 4): (sp.sqrt(2) / 2, -sp.sqrt(2) / 2),
    Fraction(1, 3): (sp.sqrt(3) / 2, sp.Rational(1, 2)),
    Fraction(2, 3): (sp.sqrt(3) / 2, sp.Rational(-1, 2)),
    Fraction(1, 5): (sp.sin(sp.pi / 5), sp.cos(sp.pi / 5)),
}

# Углы, на которых поле отвергало центральный NGON `building.002`. Полевой
# факт — доля π; градусы рядом это её округлённое отображение, и обратно из
# них доля уже не восстанавливается.
FIELD_RIGHT_ANGLE_NOISE = (
    ("89.9996", Fraction(4999980324, 10**10)),
    ("90.0004", Fraction(5000022083, 10**10)),
    ("90.0192", Fraction(5001068965, 10**10)),
)


def _binary64_corner(fraction: Fraction) -> tuple[sp.Expr, sp.Expr]:
    """(sin, cos) угла ровно так, как он приходит из binary64 координат."""

    radians = float(fraction) * math.pi
    return (
        sp.Rational(str(math.sin(radians))),
        sp.Rational(str(math.cos(radians))),
    )


def _true_fraction_of_pi(sine: sp.Expr, cosine: sp.Expr) -> Fraction:
    """Независимая эталонная величина `atan2(|sin|, cos)/π` в 60 знаках."""

    return Fraction(
        str(sp.N(sp.atan2(abs(sine), cosine) / sp.pi, 60))
    )


def _measure(phi, delta) -> CertifiedReflexAngleMeasureV1:
    return CertifiedReflexAngleMeasureV1(
        phi,
        delta,
        TurnOrientation.CCW_IN_OWNER_PATCH_ORIENTATION,
        AngleNormalizationLaw.VALUE_OVER_SYMBOLIC_PI_V1,
    )


# --------------------------------------------------------------------------
# Точный путь не сдвинулся
# --------------------------------------------------------------------------


def test_exact_right_angle_digest_is_frozen():
    """Дайджест прямого угла — тот же, что и до появления оболочки.

    Прямой угол — рабочая лошадь принятого корпуса. Если это число поедет,
    значит быстрый путь разошёлся с точным, и это дефект, а не повод обновить
    эталон.
    """

    phi, delta = angle_measure.reflex_angle_intervals_over_pi(
        *EXACT_CORNERS[Fraction(1, 2)]
    )
    digest = sha256(canonical_json_bytes(_measure(phi, delta))).hexdigest()
    assert digest == (
        "fecd969c82bc1ba25d08440a9bbf13d1c10d28e8c8c82736795c9434aea2e1e8"
    )


@pytest.mark.parametrize("fraction", [Fraction(1, 2), Fraction(1, 4), Fraction(3, 4)])
def test_exact_fraction_collapses_the_interval_to_itself(fraction: Fraction):
    """Точная дробь остаётся точкой: оболочка её не размывает."""

    sine, cosine = EXACT_CORNERS[fraction]
    phi, delta = angle_measure.reflex_angle_intervals_over_pi(sine, cosine)
    assert Fraction(delta.lower) == Fraction(delta.upper) == fraction
    assert Fraction(phi.lower) == Fraction(phi.upper) == 1 + fraction
    assert delta.absolute_error_bound == Decimal(0)


def test_exact_two_pi_is_still_refused():
    """2π — Terminal/Junction, а не Angular. Отказ именованный."""

    with pytest.raises(angle_measure.CertifiedAngleUnavailable, match="2.pi"):
        angle_measure.reflex_angle_intervals_over_pi(
            sp.Integer(0), sp.Integer(-1)
        )


# --------------------------------------------------------------------------
# Эквивалентность оболочки и точного пути
# --------------------------------------------------------------------------


@pytest.mark.parametrize("fraction", sorted(EXACT_CORNERS))
def test_enclosure_contains_the_exact_fraction(fraction: Fraction):
    """Там, где точный путь знает ответ, оболочка обязана его накрывать.

    Это и есть условие законности замены: оболочка не сдвигает угол, она лишь
    перестаёт требовать, чтобы он записывался дробью.
    """

    sine, cosine = EXACT_CORNERS[fraction]
    phi, delta = angle_measure.certified_intervals_over_pi(sine, cosine)
    assert Fraction(delta.lower) <= fraction <= Fraction(delta.upper)
    assert Fraction(phi.lower) <= 1 + fraction <= Fraction(phi.upper)


@pytest.mark.parametrize("fraction", sorted(EXACT_CORNERS))
def test_enclosure_is_tight_to_the_last_recorded_digit(fraction: Fraction):
    """Оболочка шире точки не более чем на две единицы последнего знака.

    Иначе «сертифицированный» интервал перестал бы разрешать профиль: полосы
    выбора k имеют ширину 1/3, но соседние углы отличаются на 1e-5.
    """

    sine, cosine = EXACT_CORNERS[fraction]
    _, delta = angle_measure.certified_intervals_over_pi(sine, cosine)
    unit = Fraction(1, 10**angle_measure.DECIMALS)
    assert Fraction(delta.upper) - Fraction(delta.lower) <= 2 * unit


@pytest.mark.parametrize("degrees,fraction", FIELD_RIGHT_ANGLE_NOISE)
def test_enclosure_brackets_the_independently_computed_value(
    degrees: str, fraction: Fraction
):
    """Истинное значение внутри оболочки, эталон посчитан другим способом."""

    sine, cosine = _binary64_corner(fraction)
    phi, delta = angle_measure.reflex_angle_intervals_over_pi(sine, cosine)
    truth = _true_fraction_of_pi(sine, cosine)
    assert Fraction(delta.lower) <= truth <= Fraction(delta.upper)
    assert Fraction(phi.lower) <= 1 + truth <= Fraction(phi.upper)


@pytest.mark.parametrize("path", ["reflex_angle_intervals_over_pi", "certified_intervals_over_pi"])
@pytest.mark.parametrize("fraction", sorted(EXACT_CORNERS))
def test_delta_is_exactly_phi_minus_one(path: str, fraction: Fraction):
    """δ = φ − 1 побитово, обоими путями. Этого требует валидация ядра.

    Оба интервала обязаны приходить из одного вычисления: иначе сверка
    согласованности начнёт сравнивать разное с разным.
    """

    sine, cosine = EXACT_CORNERS[fraction]
    phi, delta = getattr(angle_measure, path)(sine, cosine)
    assert delta.lower == phi.lower - Decimal(1)
    assert delta.upper == phi.upper - Decimal(1)
    assert delta.lower_kind is phi.lower_kind
    assert delta.upper_kind is phi.upper_kind


# --------------------------------------------------------------------------
# Полевые углы принимаются
# --------------------------------------------------------------------------


@pytest.mark.parametrize("degrees,expected", FIELD_RIGHT_ANGLE_NOISE)
def test_field_right_angle_noise_is_admitted(degrees: str, expected: Fraction):
    """Шум координат на прямом угле больше не отвергает патч.

    Доли взяты из полевого прогона `building.002`; совпадения требуется девять
    знаков — дальше расходятся сами координаты, а не измерение.
    """

    sine, cosine = _binary64_corner(expected)
    phi, delta = angle_measure.reflex_angle_intervals_over_pi(sine, cosine)
    assert angle_measure.rational_intervals_over_pi(
        angle_measure.angular_fraction_of_pi(sine, cosine)
    ) is None
    assert abs(Fraction(delta.lower) - expected) < Fraction(1, 10**9)
    assert _interval_strictly_above(phi, Decimal(1))
    assert _interval_strictly_below(phi, Decimal(2))


@pytest.mark.parametrize("degrees,fraction", FIELD_RIGHT_ANGLE_NOISE)
def test_field_right_angle_noise_resolves_a_profile(
    degrees: str, fraction: Fraction
):
    """Принять угол мало — по нему обязан выбираться профиль.

    Все три полевых угла лежат внутри полосы k = 1 (1/3, 2/3], далеко от её
    границ, поэтому оболочка шириной 1e-28 выбирает k однозначно.
    """

    sine, cosine = _binary64_corner(fraction)
    phi, delta = angle_measure.reflex_angle_intervals_over_pi(sine, cosine)
    assert _resolve_hidden_edge_count(_measure(phi, delta)) == 1


def test_exact_sixty_degree_corner_is_measured_but_leaves_k_undecided():
    """δ = 1/3 — сама граница полосы выбора k, и это именованная неопределённость.

    Раньше такой угол ронял патч ещё на измерении: `scaleb` округляет φ ∈ (1, 2)
    до 28 значащих цифр, φ и δ расходились, и сверка давала отказ. Теперь угол
    измерен оболочкой, а нерешаемым остаётся только то, что нерешаемо по сути:
    интервал, накрывающий границу полосы, не доказывает ни k = 0, ни k = 1.
    Ответ на это — `ANGULAR_PROFILE_SELECTION_UNCERTAIN`, а не отказ измерять.
    """

    sine, cosine = EXACT_CORNERS[Fraction(1, 3)]
    phi, delta = angle_measure.reflex_angle_intervals_over_pi(sine, cosine)
    assert Fraction(delta.lower) < Fraction(1, 3) < Fraction(delta.upper)
    assert _resolve_hidden_edge_count(_measure(phi, delta)) is None


# --------------------------------------------------------------------------
# Отказ там, где оболочка ничего не доказала
# --------------------------------------------------------------------------


def test_enclosure_refuses_when_it_cannot_separate_phi_from_2pi():
    """Неразделённая граница — именованный отказ, а не выбор ближайшего.

    Синус здесь порядка 1e-30: он не ноль, поэтому точный путь не видит 2π,
    но и от 2π он не отделён на 28 знаках.
    """

    with pytest.raises(
        angle_measure.CertifiedAngleUnavailable, match="does not prove"
    ):
        angle_measure.reflex_angle_intervals_over_pi(
            sp.sqrt(2) / 10**30, sp.Integer(-1)
        )


def test_enclosure_refuses_when_the_expression_has_no_interval_rule():
    """Выражение, для которого правила нет, отвергается по имени, а не по нулю."""

    with pytest.raises(
        angle_measure.CertifiedAngleUnavailable, match="no certified interval"
    ):
        angle_measure.reflex_angle_intervals_over_pi(
            sp.Symbol("t"), sp.Integer(1)
        )


def test_interval_endpoints_stay_closed():
    """Замкнутость границ — часть контракта: валидация сравнивает и виды тоже."""

    sine, cosine = _binary64_corner(FIELD_RIGHT_ANGLE_NOISE[0][1])
    phi, delta = angle_measure.reflex_angle_intervals_over_pi(sine, cosine)
    for interval in (phi, delta):
        assert interval.lower_kind is IntervalEndpointKind.CLOSED
        assert interval.upper_kind is IntervalEndpointKind.CLOSED


# --------------------------------------------------------------------------
# Чем именно оболочка держится за mpmath
# --------------------------------------------------------------------------


def test_mpmath_interval_endpoints_stay_exactly_readable():
    """`_mpi_` + `libmp.to_rational` — приватный контракт mpmath, и он держит всё.

    Точного публичного способа прочитать границу у mpmath нет: `.a`/`.b`
    отдают вырожденный интервал, а не число, а `float()` срезал бы разряды и
    превратил сертификат в оценку. Поэтому версия mpmath, которая уберёт
    `_mpi_` или сменит его форму, обязана падать здесь — громко и по имени, а
    не тихой потерей точности внутри `_enclose_fraction_of_pi`.
    """

    saved = iv.prec
    iv.prec = 200
    try:
        representable = iv.mpf(1) / iv.mpf(4)
        irreducible = iv.mpf(1) / iv.mpf(3)
    finally:
        iv.prec = saved

    assert hasattr(representable, "_mpi_"), (
        "mpmath больше не отдаёт `_mpi_`. Оболочка читала границы через него; "
        "нужен другой точный доступ, приблизительный не годится."
    )
    assert len(representable._mpi_) == 2

    # Точно представимое значение читается точно обеими границами.
    assert {
        angle_measure._endpoint(item) for item in representable._mpi_
    } == {Fraction(1, 4)}

    # Непредставимое — двоичными дробями строго по обе стороны от истины,
    # в порядке (нижняя, верхняя).
    lower, upper = (angle_measure._endpoint(item) for item in irreducible._mpi_)
    assert lower < Fraction(1, 3) < upper
    for endpoint in (lower, upper):
        denominator = endpoint.denominator
        assert denominator & (denominator - 1) == 0, (
            f"граница {endpoint} не двоичная дробь — чтение перестало быть точным"
        )
    assert upper - lower < Fraction(1, 2**150)
