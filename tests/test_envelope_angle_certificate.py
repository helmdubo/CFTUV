"""Сертификат вогнутого угла: δ выводится из φ, и байты проверяются как байты.

Полевой факт среза. Пересборка полевой фикстуры (`building.002`, patch 0,
выделение рёбер 2/3/7) дала снапшот с четырьмя вогнутыми углами, и
`compile_reference_envelopes` отверг его ЦЕЛИКОМ:

    REFERENCE_INPUT_CONTRACT_INVALID:
    reflex_angle_certificates.…:reflex_excess_over_pi:
    delta interval must equal phi_over_pi minus one exactly

Два угла из четырёх были точные (φ/π = 3/2 ровно), два — зашумлённые:
φ/π = 1.5000002407571825… и 1.5001068965084400… с оболочками шириной ~1e-28.
Именно зашумлённые и отвергались.

Механизм оказался не в измерении, а в ЗАПИСИ. `codec._canonical_decimal`
вызывает `Decimal.normalize()` — операцию контекста; φ ∈ (1, 2) с 28 знаками
после точки это 29 значащих цифр, и контекст с `prec = 28` округляет её при
записи, тогда как δ < 1 с теми же 28 знаками — ровно 28 значащих цифр и
переживает запись без потерь. Поэтому в памяти `δ == φ − 1` держалось, а в
выпущенных байтах — нет: расхождение в последней цифре.

Здесь заморожены обе половины: расхождение старой парной сборки (φ и δ приходят
двумя интервалами из ядра и записываются независимо) и отсутствие расхождения у
сборки через `certified_reflex_measure`, где δ выводится вычитанием единицы из
УЖЕ канонизованной φ. Точный угол 3/2 идёт контролем: его дайджест — часть
принятого корпуса и не сдвигается.
"""

from __future__ import annotations

from dataclasses import replace
from decimal import Decimal, getcontext
from hashlib import sha256
from pathlib import Path
import sys

from mathutils import Vector
import pytest
import sympy as sp


KERNEL_SRC = Path(__file__).resolve().parents[1] / "kernel" / "src"
if str(KERNEL_SRC) not in sys.path:
    sys.path.insert(0, str(KERNEL_SRC))

# Фикстура вогнутого угла живёт одним экземпляром: копия полевой заготовки в
# этом файле была бы вторым определением одной и той же фигуры.
from test_envelope_host_adapter import _reflex_bundle  # noqa: E402

from cftuv.envelope_angle_certificate import (  # noqa: E402
    SnapshotExportRejected,
    canonical_significant_digits,
    certified_reflex_measure,
    validate_exported_snapshot,
)
from cftuv.envelope_debug_profile import (  # noqa: E402
    ANGULAR_STAGE_COUNTERS,
    EnvelopeDebugProfileBuilderV1,
)
from cftuv.envelope_host_adapter import (  # noqa: E402
    build_envelope_analysis_snapshot,
)
from cftuv.model import BoundaryCorner, CornerKind  # noqa: E402
import cftuv_envelope as kernel  # noqa: E402
from cftuv_envelope.reference import angle_measure  # noqa: E402
from cftuv_envelope.reference import validation as reference_validation  # noqa: E402
from cftuv_envelope.reference.metric import ExactPlanarMetric  # noqa: E402
from cftuv_envelope.reference.planar_types import (  # noqa: E402
    ExactPlanarVector,
)
from cftuv_envelope.reference.validation import (  # noqa: E402
    _interval_contains_oriented_support_delta,
    _validate_angle_certificates,
)


# Полевые углы среза. Пара целых — это `(cosine, sine)` в том виде, в каком их
# подаёт метрика: `atan2` инвариантен к положительному масштабу, поэтому
# нормировать их незачем. Числа подобраны так, чтобы доля π совпала с полевой
# до последнего записанного знака — сама доля закреплена рядом.
FIELD_REFLEX_CORNERS = (
    (
        -75636099584,
        10**17,
        "1.5000002407571824996430586437",
        "1.5000002407571824996430586438",
    ),
    (
        -33582529823414,
        10**17,
        "1.5001068965084399994914804714",
        "1.5001068965084399994914804715",
    ),
)

# Точный прямой угол: рабочая лошадь принятого корпуса. Дайджест взят из
# `kernel/tests/test_angle_measure_certificate.py` и здесь служит контролем —
# хостовая сборка меры обязана дать те же байты, что и парная.
EXACT_RIGHT_ANGLE_DIGEST = (
    "fecd969c82bc1ba25d08440a9bbf13d1c10d28e8c8c82736795c9434aea2e1e8"
)


def _field_terms(cosine_value: int, sine_value: int):
    return sp.Integer(sine_value), sp.Integer(cosine_value)


def _paired_measure(sine, cosine, orientation):
    """Старая сборка: φ и δ приходят двумя интервалами и пишутся независимо."""

    phi, delta = angle_measure.reflex_angle_intervals_over_pi(sine, cosine)
    return kernel.CertifiedReflexAngleMeasureV1(
        phi,
        delta,
        orientation,
        kernel.AngleNormalizationLaw.VALUE_OVER_SYMBOLIC_PI_V1,
    )


def _reflex_snapshot(*, collinear_corner: bool = False, profile=None):
    bundle = _reflex_bundle(1)
    if collinear_corner:
        # Вершина 3 лежит на прямой между 2 и 4: объявленный там угол
        # коллинеарен по построению, и шаг 6 обязан его пропустить.
        loop = next(iter(bundle.patch_graph.nodes.values())).boundary_loops[0]
        loop.corners.append(
            BoundaryCorner(
                loop_vert_index=3,
                vert_index=3,
                vert_co=Vector(loop.vert_cos[3]),
                prev_chain_index=2,
                next_chain_index=3,
                corner_kind=CornerKind.GEOMETRIC,
                turn_angle_deg=0.0,
            )
        )
    return build_envelope_analysis_snapshot(bundle, profile=profile)


def _with_measure(snapshot, measure):
    certificate = next(iter(snapshot.reflex_angle_certificates))
    return replace(
        snapshot,
        reflex_angle_certificates=frozenset(
            {replace(certificate, measure_payload=measure)}
        ),
    )


def _delta_issues(snapshot):
    return tuple(
        item
        for item in kernel.validate_analysis_snapshot(snapshot)
        if item.path[-1] == "reflex_excess_over_pi"
    )


def _canonical_round_trip(snapshot):
    payload = kernel.AnalysisSnapshotCodecV1.dumps(snapshot)
    return kernel.AnalysisSnapshotCodecV1.loads(payload)


# --------------------------------------------------------------------------
# Полевая конфигурация: вход закреплён числом, а не памятью
# --------------------------------------------------------------------------


@pytest.mark.parametrize(
    "cosine_value, sine_value, lower, upper", FIELD_REFLEX_CORNERS
)
def test_field_corner_reproduces_the_recorded_enclosure(
    cosine_value: int, sine_value: int, lower: str, upper: str
):
    """Вход теста — полевая доля π, а не то, что выдаст текущий код.

    Без этой привязки остальные проверки измеряли бы сами себя: любая
    перестройка меры «сходилась» бы с новыми числами.
    """

    sine, cosine = _field_terms(cosine_value, sine_value)
    phi, _ = angle_measure.reflex_angle_intervals_over_pi(sine, cosine)
    assert (phi.lower, phi.upper) == (Decimal(lower), Decimal(upper))
    assert phi.absolute_error_bound == Decimal("1E-28")
    # 29 значащих цифр — ровно то, что не переживает канонизацию.
    assert len(phi.lower.as_tuple().digits) == canonical_significant_digits() + 1


# --------------------------------------------------------------------------
# Расхождение δ: до и после
# --------------------------------------------------------------------------


@pytest.mark.parametrize(
    "cosine_value, sine_value, lower, upper", FIELD_REFLEX_CORNERS
)
def test_paired_delta_diverges_from_phi_in_the_written_bytes(
    cosine_value: int, sine_value: int, lower: str, upper: str
):
    """Полевой отказ: в памяти сходится, в байтах — нет.

    Это диагноз, а не желаемое поведение: тест обязан покраснеть, если парная
    сборка вдруг начнёт переживать запись, потому что тогда изменился кодек, и
    вывод про механизм надо перепроверять.
    """

    snapshot = _reflex_snapshot()
    orientation = next(
        iter(snapshot.reflex_angle_certificates)
    ).measure_payload.orientation
    sine, cosine = _field_terms(cosine_value, sine_value)
    candidate = _with_measure(
        snapshot, _paired_measure(sine, cosine, orientation)
    )
    assert _delta_issues(candidate) == ()
    issues = _delta_issues(_canonical_round_trip(candidate))
    assert [item.message for item in issues] == [
        "delta interval must equal phi_over_pi minus one exactly"
    ]


@pytest.mark.parametrize(
    "cosine_value, sine_value, lower, upper", FIELD_REFLEX_CORNERS
)
def test_derived_delta_survives_the_written_bytes(
    cosine_value: int, sine_value: int, lower: str, upper: str
):
    """δ, выведенная из канонизованной φ, равна φ − 1 и в памяти, и в байтах."""

    snapshot = _reflex_snapshot()
    orientation = next(
        iter(snapshot.reflex_angle_certificates)
    ).measure_payload.orientation
    sine, cosine = _field_terms(cosine_value, sine_value)
    measure = certified_reflex_measure(
        kernel, angle_measure, sine, cosine, orientation
    )
    candidate = _with_measure(snapshot, measure)
    assert _delta_issues(candidate) == ()
    decoded = _canonical_round_trip(candidate)
    assert _delta_issues(decoded) == ()
    # Настоящее свойство, ради которого всё это: сама оболочка переживает
    # запись. У парной сборки φ на записи схлопывалась в точку, которая
    # истинного значения уже не накрывала, и заметить это могло только
    # равенство `δ == φ − 1`.
    assert (
        next(iter(decoded.reflex_angle_certificates)).measure_payload
        == measure
    )
    # Оболочка расширена наружу, а не сдвинута: полевая φ внутри неё целиком.
    assert measure.phi_over_pi.lower <= Decimal(lower)
    assert measure.phi_over_pi.upper >= Decimal(upper)
    assert measure.phi_over_pi.lower < measure.phi_over_pi.upper


def test_exact_right_angle_measure_keeps_the_accepted_digest():
    """Контроль: точный угол 3/2 не сдвинулся ни на бит.

    Два точных угла полевой фикстуры проходили и до среза. Если их байты
    поедут, значит починка зашумлённых углов оплачена принятым корпусом.
    """

    orientation = kernel.TurnOrientation.CCW_IN_OWNER_PATCH_ORIENTATION
    measure = certified_reflex_measure(
        kernel, angle_measure, sp.Integer(1), sp.Integer(0), orientation
    )
    digest = sha256(kernel.canonical_json_bytes(measure)).hexdigest()
    assert digest == EXACT_RIGHT_ANGLE_DIGEST
    assert measure.phi_over_pi.lower == measure.phi_over_pi.upper
    assert measure.reflex_excess_over_pi.lower == Decimal("0.5")


def test_canonical_widening_refuses_instead_of_leaving_the_range():
    """Расширение наружу может дойти до π — и тогда это именованный отказ.

    Оболочка шириной в один разряд у самой границы после канонизации перестаёт
    доказывать `π < φ`. Молчать здесь нельзя: сертификат, который ничего не
    доказывает, хуже отсутствующего.
    """

    edge = kernel.CertifiedDecimalIntervalV1(
        Decimal("1.0000000000000000000000000001"),
        Decimal("1.0000000000000000000000000002"),
        kernel.IntervalEndpointKind.CLOSED,
        kernel.IntervalEndpointKind.CLOSED,
        Decimal("1E-28"),
    )

    class _Stub:
        CertifiedAngleUnavailable = angle_measure.CertifiedAngleUnavailable

        @staticmethod
        def reflex_angle_intervals_over_pi(sine, cosine):
            return edge, edge

    with pytest.raises(
        angle_measure.CertifiedAngleUnavailable, match="canonical phi"
    ):
        certified_reflex_measure(
            kernel,
            _Stub,
            sp.Integer(1),
            sp.Integer(0),
            kernel.TurnOrientation.CCW_IN_OWNER_PATCH_ORIENTATION,
        )


# --------------------------------------------------------------------------
# Гипотеза среза: не отвергнет ли ядро расширенную оболочку по другой причине
# --------------------------------------------------------------------------


def test_widened_enclosure_still_contains_the_exact_oriented_turn():
    """`_validate_angle_certificates` сверяет δ с точным поворотом опор.

    Расширение наружу этой сверке помочь не может и помешать тоже: интервал
    только растёт. Проверяется на настоящей метрике полевой заготовки, а не на
    единичной — у неё грам не единичный, и угол там не евклидов.
    """

    snapshot = _reflex_snapshot()
    metric = ExactPlanarMetric.from_descriptor(
        next(iter(snapshot.surface_metric_descriptors))
    )
    orientation = kernel.TurnOrientation.CCW_IN_OWNER_PATCH_ORIENTATION
    incoming = ExactPlanarVector.from_values(sp.Integer(1), sp.Integer(0))
    for cosine_value, sine_value, _, _ in FIELD_REFLEX_CORNERS:
        outgoing = ExactPlanarVector.from_values(
            sp.Integer(cosine_value), sp.Integer(sine_value)
        )
        cosine, sine = metric.angle_g(incoming, outgoing)
        measure = certified_reflex_measure(
            kernel, angle_measure, sine, cosine, orientation
        )
        assert _interval_contains_oriented_support_delta(
            metric,
            incoming,
            outgoing,
            orientation,
            measure.reflex_excess_over_pi,
        )


def test_host_snapshot_passes_the_reference_angle_certificate_check():
    """Тот же сертификат, что уходит в поле, проходит сверку ядра целиком."""

    snapshot = _reflex_snapshot()
    domain_id = next(iter(snapshot.patch_domains)).patch_domain_id
    metric = ExactPlanarMetric.from_descriptor(
        next(iter(snapshot.surface_metric_descriptors))
    )
    assert _validate_angle_certificates(snapshot, domain_id, metric) == ()
    certificate = next(iter(snapshot.reflex_angle_certificates))
    sector = next(iter(snapshot.angular_owner_sectors))
    # Порядок концов и знак ориентации — две величины, которые ядро сверяет
    # отдельно от δ; они обязаны приходить из одного источника.
    assert certificate.measure_payload.orientation is sector.turn_orientation
    assert (
        certificate.measure_payload.reflex_excess_over_pi.lower
        <= certificate.measure_payload.reflex_excess_over_pi.upper
    )


def test_legacy_omission_and_explicit_density_select_distinct_predicates(
    monkeypatch,
):
    """Omitted keyword остаётся legacy, Density выбирается только явно."""

    snapshot = _reflex_snapshot()
    domain_id = next(iter(snapshot.patch_domains)).patch_domain_id
    metric = ExactPlanarMetric.from_descriptor(
        next(iter(snapshot.surface_metric_descriptors))
    )
    calls = []

    def legacy(*args):
        del args
        calls.append("LEGACY")
        return True

    def density(*args):
        del args
        calls.append("DENSITY")
        return True

    monkeypatch.setattr(
        reference_validation,
        "_interval_contains_oriented_support_delta",
        legacy,
    )
    monkeypatch.setattr(
        reference_validation,
        "_density_interval_contains_oriented_support_delta",
        density,
    )

    assert _validate_angle_certificates(snapshot, domain_id, metric) == ()
    legacy_count = len(calls)
    assert legacy_count > 0
    assert calls == ["LEGACY"] * legacy_count

    assert (
        _validate_angle_certificates(
            snapshot,
            domain_id,
            metric,
            density_bounded=True,
        )
        == ()
    )
    assert calls == [
        *(["LEGACY"] * legacy_count),
        *(["DENSITY"] * legacy_count),
    ]


# --------------------------------------------------------------------------
# Экспорт не выпускает невалидное
# --------------------------------------------------------------------------


def test_export_gate_rejects_a_snapshot_that_only_the_bytes_break():
    """Дыра среза: экспортёр проверял объект, а выпускал байты.

    Мера ниже валидна в памяти и накрывает точный поворот опор — по геометрии
    придраться не к чему. Ломается она ровно на записи: у φ 29 значащих цифр,
    и `normalize()` съедает последнюю. HEAD такие байты выпускал молча.
    """

    snapshot = _reflex_snapshot()
    orientation = next(
        iter(snapshot.reflex_angle_certificates)
    ).measure_payload.orientation
    phi = kernel.CertifiedDecimalIntervalV1(
        Decimal("1.4999999999999999999999999999"),
        Decimal("1.5000000000000000000000000001"),
        kernel.IntervalEndpointKind.CLOSED,
        kernel.IntervalEndpointKind.CLOSED,
        Decimal("2E-28"),
    )
    excess = kernel.CertifiedDecimalIntervalV1(
        phi.lower - Decimal(1),
        phi.upper - Decimal(1),
        kernel.IntervalEndpointKind.CLOSED,
        kernel.IntervalEndpointKind.CLOSED,
        Decimal("2E-28"),
    )
    candidate = _with_measure(
        snapshot,
        kernel.CertifiedReflexAngleMeasureV1(
            phi,
            excess,
            orientation,
            kernel.AngleNormalizationLaw.VALUE_OVER_SYMBOLIC_PI_V1,
        ),
    )
    assert kernel.validate_analysis_snapshot(candidate) == ()
    with pytest.raises(SnapshotExportRejected) as error:
        validate_exported_snapshot(kernel, candidate)
    assert error.value.outcome_name == (
        "ENVELOPE_DEBUG_ANALYSIS_SNAPSHOT_INVALID"
    )
    message = str(error.value)
    assert message.startswith("canonical bytes:")
    assert "delta interval must equal phi_over_pi minus one exactly" in message


def test_export_gate_returns_the_bytes_it_proved():
    """Приёмка возвращает выпускаемые байты, а не считает их второй раз."""

    snapshot = _reflex_snapshot()
    payload = validate_exported_snapshot(kernel, snapshot)
    assert payload == kernel.AnalysisSnapshotCodecV1.dumps(snapshot)
    assert kernel.AnalysisSnapshotCodecV1.loads(payload) == snapshot


def test_canonical_digit_budget_is_read_from_the_live_context():
    """Предел значащих цифр — свойство контекста, а не зашитое число."""

    assert canonical_significant_digits() == getcontext().prec


# --------------------------------------------------------------------------
# Наблюдаемость стадии углов
# --------------------------------------------------------------------------


def test_angular_stage_counters_separate_zero_output_from_no_corners():
    """Нулевой выход стадии обязан отличаться от «углов нет» по числам.

    До среза `ANGULAR_RELATIONS` писала только тайминг, и оба случая выглядели
    одинаково — пустотой.
    """

    profile = EnvelopeDebugProfileBuilderV1("v0-angular", "EXACT")
    snapshot = _reflex_snapshot(collinear_corner=True, profile=profile)
    counters = {
        item.name: item.value
        for item in profile.snapshot().counters
        if item.name in ANGULAR_STAGE_COUNTERS
    }
    assert counters == {
        "ANGULAR_CORNERS_CONSIDERED": 2,
        "ANGULAR_COLLINEAR_SKIPPED": 1,
        "ANGULAR_REFLEX_CORNERS": 1,
        "ANGULAR_RELATIONS_BUILT": 1,
    }
    assert len(snapshot.corner_relations) == 1


def test_angular_stage_counters_reach_the_console_profile(capsys):
    """Счётчик, попавший только в JSON, для полевого прогона не существует."""

    from cftuv.envelope_debug_renderer import _print_profile

    profile = EnvelopeDebugProfileBuilderV1("v0-angular", "EXACT")
    _reflex_snapshot(collinear_corner=True, profile=profile)
    capsys.readouterr()
    _print_profile(profile.snapshot())
    printed = capsys.readouterr().out
    for name in ANGULAR_STAGE_COUNTERS:
        assert name in printed
