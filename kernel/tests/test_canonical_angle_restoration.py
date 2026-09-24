"""Восстановление канонического авторского угла до селектора плотности.

Полевой дефект, ради которого это существует: на «одинаковых» 270°-углах одной
стены счёт скрытых рёбер расходился. Причина точная — верхняя CLOSED-граница
счётной ячейки при чётном `q` стоит РОВНО на `u = δ/π = 1/2`, а меш владельца
несёт по этому месту шум моделирования.

Числа поля берутся из слепка `artifacts/field_snapshots/wall_2_001_corner_angles.json`
и повторяются здесь ТОЧНЫМИ рациональными парами `(cos, sin²)`, вычисленными
из хорд петли обхода: 28 прямых углов, 25 из них точные, вершины 6 и 28 чуть
больше 90°, вершина 27 чуть меньше. Пары воспроизводят `dot_exact` слепка
побитово, поэтому это НЕ синтетика, похожая на поле, а само поле.

Отрицательный контроль — класс `building.003`: увод оси 0.85 мм, отклонение
3.3e-4 рад, в 48 раз за допуском намерения. Его пара `(cos, sin)` взята из
принятого корпуса (`tests/test_envelope_angle_certificate.FIELD_REFLEX_CORNERS`).
"""

from __future__ import annotations

from dataclasses import replace
from decimal import Decimal
from fractions import Fraction
from pathlib import Path

from mpmath import iv
import pytest
import sympy as sp

import cftuv_envelope as kernel
from cftuv_envelope import (
    AngularEnvelopeSpec,
    AngularProfileSelectionPolicyId,
    CanonicalAngleRestorationCertificateV1,
    CanonicalReflexAngleRelationV1,
    CertifiedDecimalIntervalV1,
    ExactAngleSymbol,
    ExactAngleV1,
    ExactRatioV1,
    IntervalEndpointKind,
    MaxSubturnParameterId,
    MaxSubturnValueId,
    ValidationCode,
    canonical_json_bytes,
)
from cftuv_envelope._authoring_intent import AUTHOR_ANGULAR_ERROR
from cftuv_envelope._canonical_angle import (
    CANONICAL_ANGLE_RESTORATION_PREDICATES,
    CANONICAL_SUBTURN_FAN_LAW,
    canonical_rotation_denominator,
    canonical_subturn_is_within_max_subturn,
    canonical_subturn_over_pi,
    CANONICAL_REFLEX_EXCESS_RELATIONS,
    PI_RATIONAL_UPPER_BOUND,
    canonical_angle_restoration_error,
    canonical_reflex_excess_restoration,
    selector_reflex_excess_interval,
)
from cftuv_envelope.contracts.metric import ExactRationalV1
from cftuv_envelope.reference import (
    ReferenceOutcome,
    compile_reference_envelopes,
)
from cftuv_envelope.reference.angle_measure import (
    reflex_angle_intervals_over_pi,
)
from cftuv_envelope.reference.common import (
    GeometryContext,
    ReferenceGeometryError,
    verify_canonical_angle_restorations,
)
from cftuv_envelope.reference.compile import (
    _resolve_angular_profile_selection,
)

import reference_factories as rf
from reference_factories import NEAR_RIGHT_ANGLE, angular_snapshot


# Точные `(cos, sin²)` углов стены 2.001. Воспроизводятся из слепка меша:
# хорды петли обхода в binary64-точных рациональных координатах, `cos` равен
# `dot_exact` слепка побитово. Класс — как назвал его слепок.
WALL_2_001_CORNERS = (
    (
        6,
        "ABOVE_90",
        Fraction(-755189, 68719476736),
        Fraction(265552190469324471433081, 4722366482869645213696),
    ),
    (
        28,
        "ABOVE_90",
        Fraction(-210521, 137438953472),
        Fraction(36930801818095871780241, 18889465931478580854784),
    ),
    (
        27,
        "BELOW_90",
        Fraction(2952919, 137438953472),
        Fraction(1058805114190427977141129, 18889465931478580854784),
    ),
    (
        9,
        "EXACTLY_90",
        Fraction(0),
        Fraction(347053852129099979241, 1180591620717411303424),
    ),
)

# Отрицательный контроль: увод 0.85 мм на `building.003`. Пара целых — это
# `(cos, sin)` в том виде, в каком её подаёт метрика; нормировать незачем,
# `atan2` инвариантен к положительному масштабу.
BUILDING_003_COSINE = -33582529823414
BUILDING_003_SINE = 10**17

_DENSITY_VALUES = (
    (0, MaxSubturnValueId.LINEAR_REFLEX_DENSITY_0_V1, ExactAngleSymbol.PI_OVER_2, 2),
    (1, MaxSubturnValueId.LINEAR_REFLEX_DENSITY_1_V1, ExactAngleSymbol.PI_OVER_3, 3),
    (2, MaxSubturnValueId.LINEAR_REFLEX_DENSITY_2_V1, ExactAngleSymbol.PI_OVER_4, 4),
    (3, MaxSubturnValueId.LINEAR_REFLEX_DENSITY_3_V1, ExactAngleSymbol.PI_OVER_5, 5),
    (4, MaxSubturnValueId.LINEAR_REFLEX_DENSITY_4_V1, ExactAngleSymbol.PI_OVER_6, 6),
)


def _measure_from_terms(cosine: sp.Expr, sine: sp.Expr):
    _, delta = reflex_angle_intervals_over_pi(sine, cosine)
    return delta


def _wall_interval(cosine: Fraction, squared_sine: Fraction):
    return _measure_from_terms(
        sp.Rational(cosine.numerator, cosine.denominator),
        sp.sqrt(sp.Rational(squared_sine.numerator, squared_sine.denominator)),
    )


def _building_003_interval():
    return _measure_from_terms(
        sp.Integer(BUILDING_003_COSINE),
        sp.Integer(BUILDING_003_SINE),
    )


class _Measure:
    """Минимальная обёртка: селектор читает только избыток и ориентацию."""

    def __init__(self, interval):
        self.reflex_excess_over_pi = interval


def _density_request(request, value_id, symbol):
    return replace(
        request,
        angular_profile_selection_policy_id=(
            AngularProfileSelectionPolicyId.HUBER_EMANATED_COUNT_DENSITY_A_V1
        ),
        max_subturn_parameter_id=(
            MaxSubturnParameterId.LINEAR_REFLEX_DENSITY_A_V1
        ),
        max_subturn_value_id=value_id,
        max_subturn_exact_value=ExactAngleV1(symbol),
    )


def _selection(request, interval):
    return _resolve_angular_profile_selection(request, _Measure(interval))


def test_pi_upper_bound_is_proven_strictly_above_pi():
    """Граница π — доказанная, а не подобранная.

    Она используется ТОЛЬКО сверху, поэтому ошибка в её сторону сузила бы
    принимаемое множество, а ошибка в другую — расширила бы допуск молча.
    """

    saved = iv.prec
    iv.prec = 256
    try:
        enclosure = iv.pi
        lower, upper = enclosure._mpi_
    finally:
        iv.prec = saved
    from mpmath import libmp

    proven_upper = Fraction(*libmp.to_rational(upper))
    proven_lower = Fraction(*libmp.to_rational(lower))
    assert proven_upper < PI_RATIONAL_UPPER_BOUND
    assert proven_lower < PI_RATIONAL_UPPER_BOUND
    # И не грубее, чем нужно: относительный запас меньше 1e-18.
    assert PI_RATIONAL_UPPER_BOUND - proven_upper < Fraction(1, 10**18)


def test_canonical_relation_set_is_the_minimal_paid_one():
    """Именованная граница карточки, а не забытое расширение.

    В множестве ровно одно отношение — прямой угол. Оно оплачено полем
    (слепок стены 2.001) и уже названо кодом: `intended_right_corner_facts`
    классифицирует «задуманно прямые» тем же допуском и других не знает.
    Семейства 45°/60° сюда не входят по решению, а не по недосмотру.
    """

    assert CANONICAL_REFLEX_EXCESS_RELATIONS == (
        (
            CanonicalReflexAngleRelationV1.CANONICAL_REFLEX_EXCESS_PI_OVER_2,
            Fraction(1, 2),
        ),
    )
    declared = {value for _, value in CANONICAL_REFLEX_EXCESS_RELATIONS}
    for absent in (Fraction(1, 4), Fraction(1, 3), Fraction(2, 3), Fraction(3, 4)):
        assert absent not in declared


@pytest.mark.parametrize(
    ("vertex", "kind", "cosine", "squared_sine"),
    WALL_2_001_CORNERS,
)
def test_wall_2_001_noise_is_inside_the_authoring_intent_tolerance(
    vertex,
    kind,
    cosine,
    squared_sine,
):
    """Число, которым решается вопрос «восстанавливать или нет»."""

    interval = _wall_interval(cosine, squared_sine)
    restoration = canonical_reflex_excess_restoration(interval)
    if kind == "EXACTLY_90":
        assert Fraction(interval.lower) == Fraction(1, 2)
        assert restoration is None
        return
    assert restoration is not None, vertex
    assert restoration.deviation_upper_bound_radians <= AUTHOR_ANGULAR_ERROR
    assert restoration.deviation_upper_bound_radians > 0
    # Слепок числит эти отклонения в 1.1e-6..2.9e-6 рад; допуск — 7e-6.
    assert float(restoration.deviation_upper_bound_radians) < 3e-6


def test_building_003_drift_is_honestly_outside_the_tolerance():
    """Отрицательный контроль: 0.85 мм — это 3.3e-4 рад, в 48 раз за допуском."""

    interval = _building_003_interval()
    deviation = max(
        abs(Fraction(interval.lower) - Fraction(1, 2)),
        abs(Fraction(interval.upper) - Fraction(1, 2)),
    )
    radians = deviation * PI_RATIONAL_UPPER_BOUND
    assert radians > AUTHOR_ANGULAR_ERROR
    assert 3.3e-4 < float(radians) < 3.4e-4
    assert canonical_reflex_excess_restoration(interval) is None
    # И сырое число идёт к селектору тем же объектом, без подмены.
    selector_interval, restoration = selector_reflex_excess_interval(interval)
    assert selector_interval is interval
    assert restoration is None


def test_wall_2_001_twins_get_one_selection_on_every_density():
    """Цель карточки на уровне сертификата селекции, все d0–d4 и legacy.

    Сравниваются ВСЕ четыре класса угла стены — два выше 90°, один ниже и
    точный близнец. После восстановления сертификат селекции у них
    ПОБИТОВО один и тот же на каждой плотности; до восстановления он
    расходился при чётном `q`.
    """

    _, legacy_request = angular_snapshot(1)
    intervals = {
        vertex: _wall_interval(cosine, squared_sine)
        for vertex, _, cosine, squared_sine in WALL_2_001_CORNERS
    }
    raw_disagreements = []
    for density, value_id, symbol, q in _DENSITY_VALUES:
        request = _density_request(legacy_request, value_id, symbol)
        canonical = {
            vertex: _selection(request, interval)
            for vertex, interval in intervals.items()
        }
        assert all(item is not None for item in canonical.values())
        counts = {
            (
                item.hidden_count,
                item.interval_certificate,
            )
            for item in canonical.values()
        }
        assert len(counts) == 1, (density, q, counts)

        # Тот же вход БЕЗ восстановления — доказательство, что тест умеет падать.
        raw = {
            vertex: _resolve_huber_bucket_without_restoration(interval, q)
            for vertex, interval in intervals.items()
        }
        if len(set(raw.values())) != 1:
            raw_disagreements.append((density, q, raw))
    # Чётные q — те самые d0/d2/d4, у которых граница ячейки сидит на 1/2.
    assert [item[0] for item in raw_disagreements] == [0, 2, 4]

    # Legacy-закон ходит той же дверью и на тех же углах не расходится вовсе.
    legacy = {
        vertex: _selection(legacy_request, interval)
        for vertex, interval in intervals.items()
    }
    assert len({item.hidden_count for item in legacy.values()}) == 1
    assert {
        vertex
        for vertex, item in legacy.items()
        if item.canonical_restoration is not None
    } == {6, 28, 27}


def _resolve_huber_bucket_without_restoration(interval, q: int):
    from cftuv_envelope.reference.compile import (
        _resolve_huber_density_bucket_interval,
    )

    return _resolve_huber_density_bucket_interval(interval, q)


def test_restoration_certificate_is_recorded_exactly_where_the_law_says():
    """Есть у восстановленных, нет у точных и нет у вне-допуска."""

    snapshot, request = angular_snapshot(NEAR_RIGHT_ANGLE)
    compiled = compile_reference_envelopes(snapshot, request)
    assert compiled.outcome is ReferenceOutcome.EXACT
    restorations = compiled.compilation.canonical_angle_restorations
    assert len(restorations) == 1
    record = next(iter(restorations))
    assert (
        record.canonical_relation
        is CanonicalReflexAngleRelationV1.CANONICAL_REFLEX_EXCESS_PI_OVER_2
    )
    assert record.canonical_reflex_excess_over_pi == ExactRatioV1(1, 2)
    assert record.tolerance_radians == ExactRationalV1(7, 10**6)
    assert record.proven_predicates == CANONICAL_ANGLE_RESTORATION_PREDICATES
    selection = next(iter(compiled.compilation.profile_selection_certificates))
    assert record.selection_certificate_id == selection.certificate_id
    assert record.corner_relation_id == selection.corner_relation_id

    for case in (0, 1, 2):
        exact_case = compile_reference_envelopes(*angular_snapshot(case))
        assert exact_case.outcome is ReferenceOutcome.EXACT
        assert exact_case.compilation.canonical_angle_restorations == frozenset()


def test_forged_restoration_is_rejected_by_the_compilation_verifier():
    """Подделка ловится в обе стороны: и лишняя запись, и снятая."""

    snapshot, request = angular_snapshot(NEAR_RIGHT_ANGLE)
    compiled = compile_reference_envelopes(snapshot, request)
    compilation = compiled.compilation
    honest = next(iter(compilation.canonical_angle_restorations))
    verify_canonical_angle_restorations(compilation)

    stripped = replace(compilation, canonical_angle_restorations=frozenset())
    with pytest.raises(ReferenceGeometryError) as removed:
        verify_canonical_angle_restorations(stripped)
    assert (
        removed.value.outcome
        is ReferenceOutcome.REFERENCE_CANONICAL_ANGLE_RESTORATION_INVALID
    )
    assert "restorable but not recorded" in str(removed.value)

    # Запись на точном угле: восстанавливать было нечего.
    exact_snapshot, exact_request = angular_snapshot(1)
    exact = compile_reference_envelopes(exact_snapshot, exact_request).compilation
    exact_selection = next(iter(exact.profile_selection_certificates))
    planted = replace(
        honest,
        selection_certificate_id=exact_selection.certificate_id,
        corner_relation_id=exact_selection.corner_relation_id,
        reflex_angle_certificate_id=(
            exact_selection.reflex_angle_certificate_id
        ),
    )
    with pytest.raises(ReferenceGeometryError) as planted_error:
        verify_canonical_angle_restorations(
            replace(exact, canonical_angle_restorations=frozenset({planted}))
        )
    assert "recorded but not restorable" in str(planted_error.value)

    # Раздутое отклонение: числу сертификата не верят, его пересчитывают.
    inflated = replace(
        honest,
        deviation_upper_bound_radians=ExactRationalV1(1, 10**12),
    )
    with pytest.raises(ReferenceGeometryError) as inflated_error:
        verify_canonical_angle_restorations(
            replace(
                compilation,
                canonical_angle_restorations=frozenset({inflated}),
            )
        )
    assert "deviation bound is not the proven one" in str(inflated_error.value)

    # Чужой сырой угол: запись обязана цитировать угол СВОЕГО снапшота.
    foreign = replace(
        honest,
        source_reflex_excess_over_pi=CertifiedDecimalIntervalV1(
            Decimal("0.5000001"),
            Decimal("0.5000002"),
            IntervalEndpointKind.CLOSED,
            IntervalEndpointKind.CLOSED,
            Decimal("0.0000001"),
        ),
    )
    with pytest.raises(ReferenceGeometryError) as foreign_error:
        verify_canonical_angle_restorations(
            replace(
                compilation,
                canonical_angle_restorations=frozenset({foreign}),
            )
        )
    assert "cites another source angle" in str(foreign_error.value)


def test_context_build_refuses_a_compilation_with_a_forged_restoration():
    """Растяжка стоит на общем входе геометрии, а не только в валидаторе."""

    snapshot, request = angular_snapshot(NEAR_RIGHT_ANGLE)
    compiled = compile_reference_envelopes(snapshot, request)
    compilation = compiled.compilation
    from cftuv_envelope.reference.validation import (
        validate_reference_geometry_payload,
    )

    frame, diagnostics = validate_reference_geometry_payload(
        compilation.analysis_snapshot,
        compilation.plan_key.patch_domain_id,
    )
    assert frame is not None and not diagnostics
    GeometryContext.build(compilation, frame)
    with pytest.raises(ReferenceGeometryError):
        GeometryContext.build(
            replace(compilation, canonical_angle_restorations=frozenset()),
            frame,
        )


_PERMUTATION_PROBE = """
import hashlib, sys
sys.path[:0] = [{src!r}, {tests!r}]
from cftuv_envelope import canonical_json_bytes
from cftuv_envelope.reference import compile_reference_envelopes
from reference_factories import NEAR_RIGHT_ANGLE, angular_snapshot

compilation = compile_reference_envelopes(*angular_snapshot(NEAR_RIGHT_ANGLE)).compilation
payload = canonical_json_bytes(compilation.canonical_angle_restorations)
print(hashlib.sha256(payload).hexdigest())
print(len(payload))
"""


def test_traversal_permutation_gives_bitwise_identical_restorations():
    """Перестановка обхода не двигает ни байта сертификата восстановления.

    «Перестановка обхода» здесь настоящая, а не переупаковка того же
    `frozenset` в другом порядке (та дала бы тот же объект и ничего бы не
    проверяла). Порядок обхода неупорядоченных множеств снапшота задаётся
    хэш-сидом процесса, поэтому проба идёт в ОТДЕЛЬНЫХ интерпретаторах с
    разными `PYTHONHASHSEED`: расхождение байтов означало бы, что сертификат
    зависит от того, в каком порядке ядро читало факты.
    """

    import os
    import subprocess
    import sys

    source = _PERMUTATION_PROBE.format(
        src=str(Path(__file__).resolve().parents[1] / "src"),
        tests=str(Path(__file__).resolve().parent),
    )
    results = []
    for seed in ("0", "1", "12345", "99991"):
        environment = dict(os.environ, PYTHONHASHSEED=seed)
        finished = subprocess.run(
            [sys.executable, "-c", source],
            capture_output=True,
            text=True,
            env=environment,
            check=False,
        )
        assert finished.returncode == 0, finished.stderr
        results.append(finished.stdout.strip())
    assert len(set(results)) == 1, results
    assert results[0].splitlines()[0] != ""


@pytest.mark.parametrize(
    ("density", "value_id", "symbol", "q"),
    _DENSITY_VALUES,
)
def test_map_metric_lift_takes_its_source_count_from_the_canonical_fact(
    density,
    value_id,
    symbol,
    q,
):
    """Лифт карты видит канонический факт, а не сырое число.

    Исходный счёт лифта — это счёт сертификата селекции, и он построен на
    канонической доле π. При чётном `q` сырое число дало бы ДРУГОЙ исходный
    счёт: ячейка `u = 1/2` закрыта сверху, и угол 90.0000015° падал в
    следующую. Здесь это зафиксировано числом на каждой плотности.
    """

    snapshot, legacy_request = angular_snapshot(NEAR_RIGHT_ANGLE)
    request = _density_request(legacy_request, value_id, symbol)
    compiled = compile_reference_envelopes(snapshot, request)
    if compiled.outcome is not ReferenceOutcome.EXACT:
        # Синтетическая фигура — ножевая: у q=6 ординальное окно веера
        # вырождено уже у ТОЧНОГО прямого угла. Это свойство фигуры, а не
        # закона, и селектор здесь всё равно проверен отдельным тестом.
        assert q == 6
        return
    selection = next(iter(compiled.compilation.profile_selection_certificates))
    spec = next(
        item
        for item in compiled.compilation.envelope_specs
        if isinstance(item, AngularEnvelopeSpec)
    )
    interval = snapshot_angle_interval(snapshot)
    raw_bucket = _resolve_huber_bucket_without_restoration(interval, q)
    canonical_bucket = selection.selection_interval_certificate.bucket_c
    assert canonical_bucket == _resolve_huber_bucket_without_restoration(
        selector_reflex_excess_interval(interval)[0], q
    )
    lift = getattr(spec, "evaluation_subturn_count_lift", None)
    if lift is not None:
        assert lift.source_hidden_edge_count == (
            selection.resolved_hidden_edge_count
        )
    if q % 2 == 0:
        assert raw_bucket == canonical_bucket + 1
    else:
        assert raw_bucket == canonical_bucket


def snapshot_angle_interval(snapshot):
    angle = next(iter(snapshot.reflex_angle_certificates))
    return angle.measure_payload.reflex_excess_over_pi


def test_plan_validator_binds_the_restoration_to_its_selection(projections):
    """Структурная сторона плана: чужая и висячая запись отвергаются."""

    plan = next(
        item for item in projections if item.case_id == "EC0-C03"
    ).plans[0]
    selection = next(iter(plan.angular_profile_selection_certificates))
    forged = CanonicalAngleRestorationCertificateV1(
        restoration_law=kernel.CanonicalAngleRestorationLawV1.AUTHORING_INTENT_CANONICAL_ANGLE_RESTORED_V1,
        selection_certificate_id=selection.certificate_id,
        corner_relation_id=selection.corner_relation_id,
        reflex_angle_certificate_id=selection.reflex_angle_certificate_id,
        canonical_relation=(
            CanonicalReflexAngleRelationV1.CANONICAL_REFLEX_EXCESS_PI_OVER_2
        ),
        canonical_reflex_excess_over_pi=ExactRatioV1(1, 2),
        source_reflex_excess_over_pi=CertifiedDecimalIntervalV1(
            Decimal("0.5000001"),
            Decimal("0.5000002"),
            IntervalEndpointKind.CLOSED,
            IntervalEndpointKind.CLOSED,
            Decimal("0.0000001"),
        ),
        deviation_upper_bound_radians=ExactRationalV1(1, 10**6),
        tolerance_radians=ExactRationalV1(7, 10**6),
        tolerance_policy_id=(
            kernel.AngleTolerancePolicyIdV1.AUTHOR_ANGULAR_ERROR_AUTHORING_INTENT_V1
        ),
        proven_predicates=CANONICAL_ANGLE_RESTORATION_PREDICATES,
    )
    changed = replace(plan, canonical_angle_restorations=frozenset({forged}))
    issues = kernel.validate_compiled_plan(changed)
    assert not any(
        issue.code is ValidationCode.CANONICAL_ANGLE_RESTORATION
        for issue in issues
    ), "structural plan check must accept a well-formed reference"

    orphan = replace(
        forged,
        selection_certificate_id=kernel.SelectionCertificateId("no-such"),
    )
    orphan_issues = kernel.validate_compiled_plan(
        replace(plan, canonical_angle_restorations=frozenset({orphan}))
    )
    assert any(
        issue.code is ValidationCode.MISSING_REFERENCE
        for issue in orphan_issues
    )

    duplicated = replace(forged, corner_relation_id=selection.corner_relation_id)
    mismatched = replace(
        duplicated,
        corner_relation_id=kernel.CornerRelationId("another-corner"),
    )
    mismatch_issues = kernel.validate_compiled_plan(
        replace(plan, canonical_angle_restorations=frozenset({mismatched}))
    )
    assert any(
        issue.code is ValidationCode.CANONICAL_ANGLE_RESTORATION
        for issue in mismatch_issues
    )


def test_certificate_bytes_carry_every_field_the_reader_needs():
    """Сертификат читаемый: угол, отклонение, допуск и применённый канон."""

    snapshot, request = angular_snapshot(NEAR_RIGHT_ANGLE)
    compilation = compile_reference_envelopes(snapshot, request).compilation
    record = next(iter(compilation.canonical_angle_restorations))
    payload = canonical_json_bytes(record).decode("utf-8")
    for name in (
        "canonical_relation",
        "canonical_reflex_excess_over_pi",
        "source_reflex_excess_over_pi",
        "deviation_upper_bound_radians",
        "tolerance_radians",
        "tolerance_policy_id",
        "restoration_law",
    ):
        assert f'"{name}"' in payload
    assert canonical_angle_restoration_error(
        record,
        snapshot_angle_interval(snapshot),
    ) is None


def test_near_right_angle_case_matches_the_field_deviation_class():
    """Фигура карточки сидит в том же классе, что и углы стены 2.001."""

    _, _, bounds = rf._ANGULAR_CASES[NEAR_RIGHT_ANGLE]
    interval = CertifiedDecimalIntervalV1(
        Decimal(bounds[0]),
        Decimal(bounds[1]),
        IntervalEndpointKind.CLOSED,
        IntervalEndpointKind.CLOSED,
        Decimal(bounds[1]) - Decimal(bounds[0]),
    )
    restoration = canonical_reflex_excess_restoration(interval)
    assert restoration is not None
    radians = float(restoration.deviation_upper_bound_radians)
    assert 1.4e-6 < radians < 1.6e-6
    wall = [
        float(
            canonical_reflex_excess_restoration(
                _wall_interval(cosine, squared_sine)
            ).deviation_upper_bound_radians
        )
        for _, kind, cosine, squared_sine in WALL_2_001_CORNERS
        if kind != "EXACTLY_90"
    ]
    assert min(wall) < radians < max(wall)


# --- Гарантия подшага на ВОССТАНОВЛЕННОМ угле -------------------------------
#
# Решение владельца после измеренной развилки: обещание «канонические 90°
# всегда дают один счёт» держится, а значит и осуществимость подшага
# оценивается против канонического угла. Это СМЕНА ВЛАСТИ, поэтому у неё своё
# имя (`SUBTURN_GUARANTEE_ON_CANONICAL_SUPPORTS_V1`), своя запись
# (`CanonicalSubturnFanAuthorityV1`) и своя цена, названная числом.


def _density_compile(case, value_id, symbol):
    snapshot, legacy_request = angular_snapshot(case)
    return snapshot, compile_reference_envelopes(
        snapshot,
        _density_request(legacy_request, value_id, symbol),
    )


def _exact_twin_case():
    """Точный прямой угол в той же фигуре: нормаль исходящей опоры ровно (0,-1)."""

    rf._ANGULAR_CASES.setdefault(
        "exact-right-angle",
        ((0.0, -1.0), (-5.0, 0.0), ("0.5", "0.5")),
    )
    return "exact-right-angle"


def test_guarantee_law_names_both_promises_separately():
    """Старое обещание остаётся под старым именем — иначе это тихое расширение."""

    assert {item.value for item in kernel.SubturnGuaranteeLawV1} == {
        "SUBTURN_ON_SOURCE_SUPPORTS_V1",
        "SUBTURN_GUARANTEE_ON_CANONICAL_SUPPORTS_V1",
    }
    assert CANONICAL_SUBTURN_FAN_LAW is (
        kernel.SubturnGuaranteeLawV1.SUBTURN_GUARANTEE_ON_CANONICAL_SUPPORTS_V1
    )


@pytest.mark.parametrize(("density", "value_id", "symbol", "q"), _DENSITY_VALUES)
def test_canonical_subturn_is_exactly_within_the_declared_maximum(
    density,
    value_id,
    symbol,
    q,
):
    """Цель нового закона — `pi/q` на каноническом угле, целочисленно."""

    hidden_count = max(1, _resolve_huber_bucket_without_restoration(
        _CANONICAL_HALF, q
    ) - 1)
    assert canonical_subturn_is_within_max_subturn(
        Fraction(1, 2), hidden_count, q
    )
    denominator = canonical_rotation_denominator(Fraction(1, 2), hidden_count)
    assert denominator is not None and denominator >= q
    assert canonical_subturn_over_pi(Fraction(1, 2), hidden_count) == Fraction(
        1, denominator
    )


class _CanonicalHalfInterval:
    lower = Fraction(1, 2)
    upper = Fraction(1, 2)
    lower_kind = IntervalEndpointKind.CLOSED
    upper_kind = IntervalEndpointKind.CLOSED


_CANONICAL_HALF = _CanonicalHalfInterval()


@pytest.mark.parametrize(("density", "value_id", "symbol", "q"), _DENSITY_VALUES)
def test_canonical_fan_authority_is_recorded_only_where_the_source_law_failed(
    density,
    value_id,
    symbol,
    q,
):
    """Форма закона: осуществимо — молчим; неосуществимо — именованная власть.

    «Неосуществимо» — это НЕ «чётное q», а ровно `2*(H+1) == q`: только там
    канонический подшаг равен `pi/q` впритык, и авторский шум выносит сырой
    подшаг за порог. При `2*(H+1) > q` (d0, d1, d3) порог не тугой, старый
    закон проходит на сырых опорах, и власть не пишется вовсе.
    """

    _, compiled = _density_compile(NEAR_RIGHT_ANGLE, value_id, symbol)
    if compiled.outcome is not ReferenceOutcome.EXACT:
        assert q == 6  # ножевая синтетика, см. тест лифта
        return
    authorities = compiled.compilation.canonical_subturn_fan_authorities
    spec = next(
        item
        for item in compiled.compilation.envelope_specs
        if isinstance(item, AngularEnvelopeSpec)
    )
    tight = 2 * (spec.resolved_hidden_edge_count + 1) == q
    assert tight is (q in (4, 6)), (density, q)
    if tight:
        assert len(authorities) == 1, (density, q)
        authority = next(iter(authorities))
        assert authority.guarantee_law is CANONICAL_SUBTURN_FAN_LAW
        assert authority.envelope_spec_id == spec.envelope_spec_id
        assert authority.max_subturn_q == q
        assert authority.hidden_edge_count == spec.resolved_hidden_edge_count
        assert authority.canonical_subturn_over_pi == ExactRatioV1(
            1, 2 * (spec.resolved_hidden_edge_count + 1)
        )
        # Цена названа числом и ограничена допуском намерения.
        residual = Fraction(
            authority.raw_residual_upper_bound_radians.numerator,
            authority.raw_residual_upper_bound_radians.denominator,
        )
        assert 0 < residual <= AUTHOR_ANGULAR_ERROR
    else:
        assert authorities == frozenset(), (density, q)


def test_even_q_fan_of_a_restored_corner_has_the_shape_of_its_exact_twin():
    """Главное: счёт, число опор и их вид совпадают с точным близнецом."""

    twin = _exact_twin_case()
    for value_id, symbol, q in (
        (MaxSubturnValueId.LINEAR_REFLEX_DENSITY_2_V1, ExactAngleSymbol.PI_OVER_4, 4),
    ):
        _, restored = _density_compile(NEAR_RIGHT_ANGLE, value_id, symbol)
        _, exact = _density_compile(twin, value_id, symbol)
        assert restored.outcome is ReferenceOutcome.EXACT
        assert exact.outcome is ReferenceOutcome.EXACT
        shapes = []
        for compiled in (restored, exact):
            spec = next(
                item
                for item in compiled.compilation.envelope_specs
                if isinstance(item, AngularEnvelopeSpec)
            )
            selection = next(
                iter(compiled.compilation.profile_selection_certificates)
            )
            shapes.append(
                (
                    selection.resolved_hidden_edge_count,
                    spec.resolved_hidden_edge_count,
                    len(spec.hidden_supports),
                    frozenset(type(item).__name__ for item in spec.hidden_supports),
                    getattr(spec, "evaluation_subturn_count_lift", None) is None,
                    tuple(
                        sorted(
                            (item.ordinal, item.turn_fraction)
                            for item in spec.hidden_supports
                        )
                    ),
                )
            )
        assert shapes[0] == shapes[1], (q, shapes)


def test_stripping_the_canonical_fan_authority_is_a_named_refusal():
    """Снятая власть — такая же подделка, как лишняя."""

    from cftuv_envelope.reference.angular import seal_angular_support_cache
    from cftuv_envelope.reference.validation import (
        validate_reference_geometry_payload,
    )

    _, compiled = _density_compile(
        NEAR_RIGHT_ANGLE,
        MaxSubturnValueId.LINEAR_REFLEX_DENSITY_2_V1,
        ExactAngleSymbol.PI_OVER_4,
    )
    compilation = compiled.compilation
    assert len(compilation.canonical_subturn_fan_authorities) == 1
    frame, diagnostics = validate_reference_geometry_payload(
        compilation.analysis_snapshot,
        compilation.plan_key.patch_domain_id,
        density_bounded=True,
    )
    assert frame is not None and not diagnostics
    seal_angular_support_cache(GeometryContext.build(compilation, frame))

    stripped = replace(
        compilation,
        canonical_subturn_fan_authorities=frozenset(),
    )
    with pytest.raises(ReferenceGeometryError) as error:
        seal_angular_support_cache(GeometryContext.build(stripped, frame))
    assert (
        error.value.outcome
        is ReferenceOutcome.REFERENCE_CANONICAL_SUBTURN_FAN_INVALID
    )
    assert "without its recorded authority" in str(error.value)


def test_canonical_fan_authority_on_a_feasible_source_fan_is_a_named_refusal():
    """Подделка гейта (г): канонический счёт при живой сырой осуществимости."""

    from cftuv_envelope.reference.angular import seal_angular_support_cache
    from cftuv_envelope.reference.validation import (
        validate_reference_geometry_payload,
    )

    _, even = _density_compile(
        NEAR_RIGHT_ANGLE,
        MaxSubturnValueId.LINEAR_REFLEX_DENSITY_2_V1,
        ExactAngleSymbol.PI_OVER_4,
    )
    _, odd = _density_compile(
        NEAR_RIGHT_ANGLE,
        MaxSubturnValueId.LINEAR_REFLEX_DENSITY_1_V1,
        ExactAngleSymbol.PI_OVER_3,
    )
    assert odd.compilation.canonical_subturn_fan_authorities == frozenset()
    authority = next(iter(even.compilation.canonical_subturn_fan_authorities))
    spec = next(
        item
        for item in odd.compilation.envelope_specs
        if isinstance(item, AngularEnvelopeSpec)
    )
    planted = replace(
        authority,
        envelope_spec_id=spec.envelope_spec_id,
        max_subturn_q=3,
    )
    forged = replace(
        odd.compilation,
        canonical_subturn_fan_authorities=frozenset({planted}),
    )
    frame, _ = validate_reference_geometry_payload(
        forged.analysis_snapshot,
        forged.plan_key.patch_domain_id,
        density_bounded=True,
    )
    with pytest.raises(ReferenceGeometryError) as error:
        seal_angular_support_cache(GeometryContext.build(forged, frame))
    assert (
        error.value.outcome
        is ReferenceOutcome.REFERENCE_CANONICAL_SUBTURN_FAN_INVALID
    )
    assert "already" in str(error.value)
