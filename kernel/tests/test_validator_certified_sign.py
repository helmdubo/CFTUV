"""Дешёвый сертифицированный путь валидатора (VALIDATOR_CERTIFIED_SIGN_V1) обязан давать тот же ответ, что точный.

`validation._interval_contains_oriented_support_delta` сначала зовёт `angle_certificate_sign.certified_oriented_support_delta`
(однородная форма с оболочками, без SymPy) и уходит в прежний точный путь (`..._exact`, тело не менялось), когда сертификата
нет. Здесь это проверяется дифференциально: на синтетических случаях с известным точным ответом, на точных границах
(закрытый против открытого конца), на разностях от 1e-28 до 1e-150 и на вырожденных входах. Исход сравнивается целиком —
значение либо имя исключения, — потому что «точный путь бросает `CertifiedPredicateUndecidable`» тоже ответ вызывающего.
"""

from __future__ import annotations

import random
from dataclasses import replace
from decimal import Decimal
from fractions import Fraction
from types import SimpleNamespace

import mpmath
import pytest
import sympy as sp

from cftuv_envelope.contracts.analysis import TurnOrientation
from cftuv_envelope.numeric import CertifiedDecimalIntervalV1, IntervalEndpointKind
from cftuv_envelope.reference import ReferenceOutcome
from cftuv_envelope.reference import angle_certificate_sign as sign_module
from cftuv_envelope.reference import symbolic_backend as backend
from cftuv_envelope.reference import validation
from cftuv_envelope.reference.metric import ExactPlanarMetric
from cftuv_envelope.reference.planar_types import ExactPlanarVector
from reference_factories import NEAR_RIGHT_ANGLE, angular_snapshot

CCW = TurnOrientation.CCW_IN_OWNER_PATCH_ORIENTATION
CW = TurnOrientation.CW_IN_OWNER_PATCH_ORIENTATION
CLOSED = IntervalEndpointKind.CLOSED
OPEN = IntervalEndpointKind.OPEN

DIGITS = 28  # столько десятичных знаков пишет хост в сертификат
UNIT = 10**DIGITS

certified = sign_module.certified_oriented_support_delta
old_predicate = validation._interval_contains_oriented_support_delta_exact
new_predicate = validation._interval_contains_oriented_support_delta


def metric_of(gram, owner_sign=1):
    entries = tuple(tuple(sp.Rational(item) for item in row) for row in gram)
    return ExactPlanarMetric(entries, entries, owner_sign)


def vector(x, y):
    return ExactPlanarVector.from_values(sp.Rational(x), sp.Rational(y))


def decimal_of(units, digits=DIGITS):
    """`Decimal("<units>E-<digits>")` — строковый конструктор не округляет по контексту, как и у хоста."""

    return Decimal(f"{units}E-{digits}")


def interval_of(lower_units, upper_units, lower_kind=CLOSED, upper_kind=CLOSED):
    return CertifiedDecimalIntervalV1(
        decimal_of(lower_units), decimal_of(upper_units), lower_kind, upper_kind, Decimal("1E-28")
    )


def outcome(predicate, *arguments):
    """Исход вызова целиком: значение либо имя исключения (точный путь бросает на недоказуемом)."""

    try:
        return ("ok", bool(predicate(*arguments)))
    except BaseException as error:  # noqa: BLE001
        return ("raised", type(error).__name__)


def angle_units(gram, first, second, digits, *, ceil=False, dps=700):
    """Целое `floor|ceil(acos(dot / sqrt(Na Nb)) / π * 10^digits)`: настоящая доля π с запасом точности, без округления контекстом."""

    (g00, g01), (g10, g11) = gram

    def norm(value):
        return value[0] * (g00 * value[0] + g01 * value[1]) + value[1] * (g10 * value[0] + g11 * value[1])

    with mpmath.workdps(dps):
        dot = first[0] * (g00 * second[0] + g01 * second[1]) + first[1] * (g10 * second[0] + g11 * second[1])
        root = mpmath.sqrt(mpmath.mpf(norm(first)) * mpmath.mpf(norm(second)))
        scaled = mpmath.acos(mpmath.mpf(dot) / root) / mpmath.pi * mpmath.mpf(10) ** digits
        return int(mpmath.ceil(scaled) if ceil else mpmath.floor(scaled))


def random_case(rng):
    """Случайные векторы, положительно определённая метрика, ориентация и интервал вокруг настоящего угла либо мимо него."""

    while True:
        first = (rng.randint(-9, 9), rng.randint(-9, 9))
        second = (rng.randint(-9, 9), rng.randint(-9, 9))
        cross = first[0] * second[1] - first[1] * second[0]
        if cross != 0:
            break
    if rng.random() < 0.3:
        gram = ((1, 0), (0, 1))
    else:
        while True:
            p, r, q = rng.randint(1, 9), rng.randint(1, 9), rng.randint(-5, 5)
            if p * r - q * q > 0:
                break
        gram = ((p, q), (q, r))
    owner_sign = rng.choice((1, -1))
    turn_sign = owner_sign * (1 if cross > 0 else -1)
    orientation = CCW if turn_sign > 0 else CW
    if rng.random() < 0.12:
        orientation = CW if orientation is CCW else CCW
    floor_units = angle_units(gram, first, second, DIGITS)
    ceil_units = angle_units(gram, first, second, DIGITS, ceil=True)
    side = rng.choice(("encloses", "encloses", "encloses", "above", "below"))
    pad = rng.choice((0, 1, 10**19, 10**26, 2 * 10**27))
    gap = rng.choice((1, 10**16, 3 * 10**26))
    width = rng.choice((0, 10**22, 10**27))
    if side == "encloses":
        low, high = floor_units - pad, ceil_units + pad
    elif side == "above":
        low = ceil_units + gap
        high = low + width
    else:
        high = floor_units - gap
        low = high - width
    kinds = (rng.choice((CLOSED, OPEN)), rng.choice((CLOSED, OPEN)))
    if low == high:
        kinds = (CLOSED, CLOSED)
    return (
        metric_of(gram, owner_sign),
        vector(*first),
        vector(*second),
        orientation,
        interval_of(low, high, *kinds),
    )


def test_random_cases_agree_with_the_exact_path_and_mostly_certify():
    rng = random.Random(20261010)
    decided = 0
    cases = 130
    for index in range(cases):
        arguments = random_case(rng)
        reference = outcome(old_predicate, *arguments)
        assert outcome(new_predicate, *arguments) == reference, index
        cheap = certified(*arguments)
        if cheap is not None:
            decided += 1
            assert ("ok", cheap) == reference, index
    # Страховка от тихого «всегда уступаю»: на неточных углах дешёвый путь обязан решать почти всё.
    assert decided >= cases * 0.8


def test_closed_and_open_endpoints_never_change_a_certified_answer():
    rng = random.Random(7)
    fully_certified = 0
    for _ in range(40):
        metric, first, second, orientation, base = random_case(rng)
        results = set()
        for lower_kind in (CLOSED, OPEN):
            for upper_kind in (CLOSED, OPEN):
                if base.lower == base.upper and (lower_kind is OPEN or upper_kind is OPEN):
                    continue
                varied = CertifiedDecimalIntervalV1(base.lower, base.upper, lower_kind, upper_kind, base.absolute_error_bound)
                cheap = certified(metric, first, second, orientation, varied)
                assert cheap is None or ("ok", cheap) == outcome(old_predicate, metric, first, second, orientation, varied)
                results.add(cheap)
        if None not in results:
            fully_certified += 1
            assert len(results) == 1
    assert fully_certified >= 20


@pytest.mark.parametrize(
    "second, tie",
    [
        ((0, 1), Decimal("0.5")),  # прямой угол: cos(π/2) = 0 ровно
        ((1, 1), Decimal("0.25")),  # 45 градусов: cos(π/4) = sqrt(2)/2 ровно
        ((-1, 1), Decimal("0.75")),  # 135 градусов
    ],
)
@pytest.mark.parametrize("lower_kind", (CLOSED, OPEN))
@pytest.mark.parametrize("upper_kind", (CLOSED, OPEN))
def test_exact_boundaries_are_left_to_the_exact_path(second, tie, lower_kind, upper_kind):
    """Точное равенство на конце: дешёвый путь уступает, а закрытый/открытый конец решает прежний код."""

    metric = metric_of(((1, 0), (0, 1)))
    first, other = vector(1, 0), vector(*second)
    away = Decimal("0.2")
    on_upper = CertifiedDecimalIntervalV1(tie - away, tie, lower_kind, upper_kind, Decimal("0"))
    on_lower = CertifiedDecimalIntervalV1(tie, tie + away, lower_kind, upper_kind, Decimal("0"))
    for case, expected in ((on_upper, upper_kind is CLOSED), (on_lower, lower_kind is CLOSED)):
        assert certified(metric, first, other, CCW, case) is None
        assert outcome(old_predicate, metric, first, other, CCW, case) == ("ok", expected)
        assert outcome(new_predicate, metric, first, other, CCW, case) == ("ok", expected)


GENERIC = {"gram": ((2, 1), (1, 3)), "first": (3, 1), "second": (-1, 2)}


def generic_arguments():
    return (
        metric_of(GENERIC["gram"]),
        vector(*GENERIC["first"]),
        vector(*GENERIC["second"]),
        CCW,
    )


def lower_endpoint_at_distance(exponent, side):
    """Нижний конец интервала на расстоянии ~10^-exponent от настоящей доли π (side=+1 выше угла, -1 ниже)."""

    digits = exponent + 8
    units = angle_units(GENERIC["gram"], GENERIC["first"], GENERIC["second"], digits)
    # Целый сдвиг в 10^8 единиц последнего знака равен 10^-exponent и не зависит от округления вниз.
    return CertifiedDecimalIntervalV1(
        decimal_of(units + side * 10**8, digits),
        Decimal("0.99"),
        CLOSED,
        CLOSED,
        Decimal("0"),
    )


@pytest.mark.parametrize("exponent", (28, 40, 60, 75))
@pytest.mark.parametrize("side", (+1, -1))
def test_small_distances_from_an_endpoint_are_certified_and_agree(exponent, side):
    """Расстояния до конца от 1e-28 (так пишет хост) до 1e-75: оболочка решает, точный путь решает так же."""

    arguments = generic_arguments()
    case = lower_endpoint_at_distance(exponent, side)
    cheap = certified(*arguments, case)
    assert cheap is not None
    # Конец выше настоящего угла (side=+1) — угол не содержится; ниже (side=-1) — содержится.
    assert cheap is (side < 0)
    assert outcome(old_predicate, *arguments, case) == ("ok", cheap)
    assert outcome(new_predicate, *arguments, case) == ("ok", cheap)


@pytest.mark.parametrize("exponent, side", [(95, +1), (110, -1), (150, +1)])
def test_distances_beyond_the_enclosure_fall_back_and_agree_with_the_exact_path(exponent, side):
    arguments = generic_arguments()
    case = lower_endpoint_at_distance(exponent, side)
    assert certified(*arguments, case) is None
    # Прежний путь может решить либо бросить `CertifiedPredicateUndecidable`: исход обязан совпасть целиком.
    assert outcome(new_predicate, *arguments, case) == outcome(old_predicate, *arguments, case)


def test_the_cheap_enclosure_stays_below_the_reach_of_the_exact_path():
    """Всё, что решает дешёвый путь, лежит в пределах досягаемости `evalf` точного пути (потолок 333 бита)."""

    from sympy.core.evalf import DEFAULT_MAXPREC

    assert sign_module._ENCLOSURE_BITS < DEFAULT_MAXPREC


def test_a_turn_against_the_expected_orientation_is_false_without_reading_the_interval():
    metric = metric_of(((1, 0), (0, 1)))
    first, second = vector(1, 0), vector(1, 2)
    unreadable = SimpleNamespace(lower=object(), upper=object(), lower_kind=CLOSED, upper_kind=CLOSED)
    assert certified(metric, first, second, CW, unreadable) is False
    assert outcome(old_predicate, metric, first, second, CW, unreadable) == ("ok", False)
    assert outcome(new_predicate, metric, first, second, CW, unreadable) == ("ok", False)


def test_degenerate_inputs_fall_back_and_behave_exactly_as_before():
    identity = metric_of(((1, 0), (0, 1)))
    window = interval_of(UNIT // 10, 9 * UNIT // 10)
    cases = {
        "zero_vector": (identity, vector(0, 0), vector(1, 1), CCW, window),
        "negative_norm": (metric_of(((-1, 0), (0, -1))), vector(1, 0), vector(0, 1), CCW, window),
        "parallel": (identity, vector(1, 1), vector(2, 2), CCW, window),
        "antiparallel": (identity, vector(1, 1), vector(-3, -3), CW, window),
        "irrational_component": (
            identity,
            ExactPlanarVector.from_values(sp.sqrt(2), sp.Integer(1)),
            vector(0, 1),
            CCW,
            window,
        ),
    }
    for name, arguments in cases.items():
        assert certified(*arguments) is None, name
        assert outcome(new_predicate, *arguments) == outcome(old_predicate, *arguments), name
    assert outcome(old_predicate, *cases["zero_vector"]) == ("raised", "ValueError")
    assert outcome(old_predicate, *cases["negative_norm"]) == ("raised", "ValueError")


def test_an_endpoint_that_does_not_read_as_a_decimal_is_left_to_the_exact_path():
    metric = metric_of(((1, 0), (0, 1)))
    first, second = vector(1, 0), vector(1, 2)
    broken = SimpleNamespace(lower="not-a-number", upper="0.5", lower_kind=CLOSED, upper_kind=CLOSED)
    assert certified(metric, first, second, CCW, broken) is None
    assert outcome(new_predicate, metric, first, second, CCW, broken) == outcome(old_predicate, metric, first, second, CCW, broken)


@pytest.mark.parametrize("mode", (backend.SymbolicBackendV1.SYMPY, backend.SymbolicBackendV1.SHADOW))
def test_only_the_native_exact_backend_uses_the_cheap_path(mode):
    """`SYMPY` — прежний путь целиком (откат), `SHADOW` — сверка двух путей: дешёвый путь в них молчит."""

    arguments = random_case(random.Random(3))
    with backend.symbolic_backend(mode):
        backend.reset_backend_counts()
        assert certified(*arguments) is None
        assert not any(key.startswith(sign_module.EVENT_SITE) for key in backend.BACKEND_COUNTS)
        assert outcome(new_predicate, *arguments) == outcome(old_predicate, *arguments)
    backend.reset_backend_counts()


def test_every_decision_is_a_named_event():
    metric = metric_of(((1, 0), (0, 1)))
    first = vector(1, 0)
    window = interval_of(UNIT // 10, 4 * UNIT // 10)
    site = sign_module.EVENT_SITE
    scenarios = (
        ((metric, first, vector(1, 2), CCW, window), f"{site}.certified"),
        ((metric, first, vector(0, 1), CCW, interval_of(UNIT // 4, UNIT // 2)), f"{site}.exact_fallback.straddles_zero"),
        ((metric, first, vector(2, 0), CCW, window), f"{site}.exact_fallback.degenerate"),
        (
            (metric, ExactPlanarVector.from_values(sp.sqrt(3), sp.Integer(1)), vector(0, 1), CCW, window),
            f"{site}.exact_fallback.outside_field",
        ),
    )
    for arguments, event in scenarios:
        backend.reset_backend_counts()
        certified(*arguments)
        assert backend.BACKEND_COUNTS == {event: 1}
    backend.reset_backend_counts()


def test_enclosures_contain_the_true_values_and_are_tight():
    rng = random.Random(11)
    for _ in range(40):
        value = Fraction(rng.randint(1, 10**30), rng.randint(1, 10**20))
        low, high = sign_module._sqrt_enclosure(value)
        assert 0 < low <= high and low * low <= value <= high * high
        assert (high - low) / high < Fraction(1, 2 ** (sign_module._ENCLOSURE_BITS - 2))
    saved = mpmath.mp.dps
    mpmath.mp.dps = 120
    try:
        for _ in range(40):
            numerator, denominator = rng.randint(-2 * UNIT, 2 * UNIT), UNIT
            low, high = sign_module._cosine_pi_enclosure(numerator, denominator)
            reference = mpmath.cos(mpmath.pi * mpmath.mpf(numerator) / denominator)
            assert mpmath.mpf(low.numerator) / low.denominator <= reference <= mpmath.mpf(high.numerator) / high.denominator
            assert high - low < Fraction(1, 2 ** (sign_module._ENCLOSURE_BITS - 8))
    finally:
        mpmath.mp.dps = saved


def test_the_cheap_path_restores_the_global_interval_precision():
    from mpmath import iv

    before = iv.prec
    certified(metric_of(((1, 0), (0, 1))), vector(1, 0), vector(1, 2), CCW, interval_of(UNIT // 10, 4 * UNIT // 10))
    assert iv.prec == before


@pytest.mark.parametrize("case", (0, 1, 2, NEAR_RIGHT_ANGLE))
def test_payload_validation_is_identical_with_and_without_the_cheap_path(case, monkeypatch):
    """Диагностики целого `validate_reference_geometry_payload`: те же значения, в том же порядке, на согласованном и на ложном сертификате."""

    snapshot, _ = angular_snapshot(case)
    domain = next(iter(snapshot.patch_domains)).patch_domain_id
    certificate = next(iter(snapshot.reflex_angle_certificates))
    incompatible = CertifiedDecimalIntervalV1(Decimal("0.0"), Decimal("0.1"), CLOSED, CLOSED, Decimal("0.10"))
    forged = replace(
        snapshot,
        reflex_angle_certificates=frozenset(
            {replace(certificate, measure_payload=replace(certificate.measure_payload, reflex_excess_over_pi=incompatible))}
        ),
    )
    for variant, mismatch in ((snapshot, False), (forged, True)):
        backend.reset_backend_counts()
        new = validation.validate_reference_geometry_payload(variant, domain)
        events = dict(backend.BACKEND_COUNTS)
        with monkeypatch.context() as patch:
            patch.setattr(validation, "_interval_contains_oriented_support_delta", old_predicate)
            old = validation.validate_reference_geometry_payload(variant, domain)
        assert new == old
        assert events.get(f"{sign_module.EVENT_SITE}.certified", 0) >= 1, events
        if mismatch:
            assert [item.outcome for item in new[1]] == [ReferenceOutcome.REFERENCE_ANGLE_SUPPORT_CERTIFICATE_MISMATCH]
        else:
            assert new[0] is not None and new[1] == ()
    backend.reset_backend_counts()
