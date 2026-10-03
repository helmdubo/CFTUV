"""Узкая полоса поворота: лучи привязаны к равному шагу, а не к ближайшей простой прямой решётки.

Полевой дефект (владелец, 2026-10-03: «в некоторых ситуациях угол 90 градусов
всё равно выстреливает какой-то рандом»): адаптивная власть искала минимальную
общую высоту в ШИРОКОМ окне Вороного — полшага веера в обе стороны, — и ставила
луч на прямую наименьшей высоты решётки карты. Для честных углов, которым таблица
канона не принадлежит (90.4–92 градуса, 72/108, 88), шаги выходили 17/32/41 вместо
30/30/30. Закон окна (`adaptive_density_band.WINDOW_LAW_NARROW_BAND`) сужает окно
до полосы `±omega` вокруг луча РАВНОУГОЛЬНОГО идеала, `tan(omega) = 1/57`
(1.005 градуса). Поиск тот же — минимальная общая высота, затем ближайший к идеалу, —
и подшаг `<= pi/q` проверяется точно по настоящим соседям.
"""

from __future__ import annotations

import math
from dataclasses import replace
from fractions import Fraction
from pathlib import Path

import pytest
import sympy as sp

import cftuv_envelope as kernel
from cftuv_envelope import AngularEnvelopeSpec
from cftuv_envelope.contracts.analysis import TurnOrientation
from cftuv_envelope.contracts.envelopes import AdaptiveMinimalRationalFanAuthorityV2
from cftuv_envelope.reference import ReferenceOutcome, compile_reference_envelopes
from cftuv_envelope.reference import compile as compile_module
from cftuv_envelope.reference import adaptive_density_band as band_module
from cftuv_envelope.reference import density_fan_binding as binding_module
from cftuv_envelope.reference.adaptive_density_band import (
    ADAPTIVE_FAN_NARROW_BAND_HALF_TANGENT,
    ADAPTIVE_FAN_NARROW_ROTATION_BAND,
    WINDOW_LAW_NARROW_BAND,
    WINDOW_LAW_VORONOI,
    authority_window_law,
    window_law_predicates,
)
from cftuv_envelope.reference.adaptive_density_fan import (
    ADAPTIVE_FAN_PROVEN_PREDICATES,
    AdaptiveDensityFanInvalid,
    DensityRationalAuthorityExhausted,
    certify_adaptive_density_fan,
    verify_adaptive_density_fan,
    verify_sealed_adaptive_density_fan,
)
from cftuv_envelope.reference.angular import (
    _interpolated_normals,
    angular_support_data,
    seal_angular_support_cache,
)
from cftuv_envelope.reference.adaptive_density_fan import _covectors, _dual_dot
from cftuv_envelope.reference.common import GeometryContext
from cftuv_envelope.reference.metric import ExactPlanarMetric
from cftuv_envelope.reference.planar_types import ExactPlanarVector
from cftuv_envelope.reference.validation import validate_reference_geometry_payload
from cftuv_envelope.wavefront import prepare_conveyor

ADAPTIVE_FAN_NARROW_BAND_PROVEN_PREDICATES = window_law_predicates(
    WINDOW_LAW_NARROW_BAND, ADAPTIVE_FAN_PROVEN_PREDICATES
)
KERNEL = Path(__file__).resolve().parents[1]
HONEST = KERNEL / "fixtures" / "binding_noise_canonical_v1" / "building004_patch0"
OMEGA_DEGREES = math.degrees(math.atan(float(ADAPTIVE_FAN_NARROW_BAND_HALF_TANGENT)))
ORIENTATION = TurnOrientation.CCW_IN_OWNER_PATCH_ORIENTATION


def _euclidean_metric() -> ExactPlanarMetric:
    return ExactPlanarMetric(
        ((sp.Rational(1), sp.Rational(0)), (sp.Rational(0), sp.Rational(1))),
        ((sp.Rational(1), sp.Rational(0)), (sp.Rational(0), sp.Rational(1))),
        1,
    )


def _ideal(incoming, outgoing, count):
    metric = _euclidean_metric()
    ideal = _interpolated_normals(
        metric,
        ExactPlanarVector.from_values(*incoming),
        ExactPlanarVector.from_values(*outgoing),
        count,
        ORIENTATION,
        huber_density=True,
    )
    return metric, ideal


def _degrees(vector) -> float:
    x, y = (float(sp.N(item, 30)) for item in vector.expressions())
    return math.degrees(math.atan2(y, x))


def _bound_degrees(authority, ordinal: int) -> float:
    """Угол привязанного луча власти. Ковектор примитивен, метрика единичная."""

    x, y = authority.bound_primitive_integer_vectors[ordinal - 1]
    return math.degrees(math.atan2(y, x))


# --------------------------------------------------------------------------
# 1. Полоса: луч в пределах omega от равного шага
# --------------------------------------------------------------------------


def test_the_band_is_one_declared_quantity():
    assert ADAPTIVE_FAN_NARROW_BAND_HALF_TANGENT == Fraction(1, 57)
    assert 1.0 < OMEGA_DEGREES < 1.01
    assert ADAPTIVE_FAN_NARROW_BAND_PROVEN_PREDICATES == (
        ADAPTIVE_FAN_PROVEN_PREDICATES | {ADAPTIVE_FAN_NARROW_ROTATION_BAND}
    )


@pytest.mark.parametrize(
    ("incoming", "outgoing", "count", "q"),
    (
        ((1, 0), (3, 1), 1, 6),   # 18.43 градуса, один луч
        ((1, 0), (0, 1), 2, 5),   # 90 градусов, лучи на 30 и 60 — иррациональны
        ((1, 0), (-1, 1), 1, 2),  # 135 градусов, луч на 67.5
        ((1, 0), (0, 1), 3, 6),   # 90 градусов, три луча на 22.5/45/67.5
    ),
)
def test_a_bound_ray_stays_within_the_band_of_the_equal_step_ideal(
    incoming, outgoing, count, q
):
    metric, ideal = _ideal(incoming, outgoing, count)
    band = certify_adaptive_density_fan(
        metric, ideal, ORIENTATION, q,
        binding_reasons=(None,) * count, window_law=WINDOW_LAW_NARROW_BAND,
    )
    voronoi = certify_adaptive_density_fan(
        metric, ideal, ORIENTATION, q, binding_reasons=(None,) * count,
    )
    assert authority_window_law(band) == WINDOW_LAW_NARROW_BAND
    assert authority_window_law(voronoi) == WINDOW_LAW_VORONOI
    assert band.proven_predicates == ADAPTIVE_FAN_NARROW_BAND_PROVEN_PREDICATES
    assert voronoi.proven_predicates == ADAPTIVE_FAN_PROVEN_PREDICATES
    worst_band = 0.0
    worst_voronoi = 0.0
    for ordinal in range(1, count + 1):
        target = _degrees(ideal[ordinal])
        worst_band = max(worst_band, abs(_bound_degrees(band, ordinal) - target))
        worst_voronoi = max(worst_voronoi, abs(_bound_degrees(voronoi, ordinal) - target))
    assert worst_band <= OMEGA_DEGREES + 1e-9, (worst_band, OMEGA_DEGREES)
    # Контроль: полоса действительно уже окна Вороного на этих данных.
    assert worst_voronoi > worst_band
    # Обе власти проверяются НЕЗАВИСИМО: полное перевыведение и запечатанная структура.
    verify_adaptive_density_fan(metric, ideal, ORIENTATION, band)
    verify_adaptive_density_fan(metric, ideal, ORIENTATION, voronoi)
    verify_sealed_adaptive_density_fan(band, ideal_count=len(ideal))


def test_a_fan_step_narrower_than_the_band_keeps_the_ray_between_the_supports(monkeypatch):
    """Мелкий излом дуги: шаг веера меньше `2*omega`, полоса — пересечение с окном Вороного.

    Без пересечения окно луча ушло бы по ту сторону входящей опоры: кандидаты там
    есть на малых высотах, но веер с ними невалиден, и поиск тратил точную работу
    до исчерпания (поле: арка, 17 углов, домен 4 с -> 30 с и четыре отказа).
    """

    metric, ideal = _ideal((1, 0), (60, 1), 1)
    inside = 0.0 < _degrees(ideal[1]) < _degrees(ideal[-1])
    assert inside and _degrees(ideal[-1]) < 2 * OMEGA_DEGREES
    band = certify_adaptive_density_fan(
        metric, ideal, ORIENTATION, 3, binding_reasons=(None,), window_law=WINDOW_LAW_NARROW_BAND,
    )
    ray = _bound_degrees(band, 1)
    assert 0.0 < ray < _degrees(ideal[-1])
    verify_adaptive_density_fan(metric, ideal, ORIENTATION, band)
    # Отрицательный контроль: без пересечения с окном Вороного полоса выходит за опору, и
    # власть либо отказывает по имени, либо не доказывается — но не ставит луч молча.
    from cftuv_envelope.reference.common import ReferenceGeometryError

    monkeypatch.setattr(band_module, "_neighbour_is_inside_the_double_band", lambda *a, **k: False)
    _, unclipped_ideal = _ideal((1, 0), (60, 1), 1)
    with pytest.raises(
        (DensityRationalAuthorityExhausted, AdaptiveDensityFanInvalid, ReferenceGeometryError)
    ):
        certify_adaptive_density_fan(
            metric, unclipped_ideal, ORIENTATION, 3,
            binding_reasons=(None,), window_law=WINDOW_LAW_NARROW_BAND,
        )


def test_the_voronoi_authority_bytes_do_not_carry_the_band_law():
    """Прежняя власть заморожена: имя закона окна — отсутствие предиката полосы."""

    metric, ideal = _ideal((1, 0), (3, 1), 1)
    voronoi = certify_adaptive_density_fan(metric, ideal, ORIENTATION, 2, binding_reasons=(None,))
    assert ADAPTIVE_FAN_NARROW_ROTATION_BAND not in voronoi.proven_predicates
    assert voronoi.proven_predicates == ADAPTIVE_FAN_PROVEN_PREDICATES


def test_a_forged_window_law_is_refused_by_the_independent_verifier():
    """Закон окна назван в предикатах и ПЕРЕСЧИТЫВАЕТСЯ: подмена имени не проходит."""

    metric, ideal = _ideal((1, 0), (0, 1), 2)
    band = certify_adaptive_density_fan(
        metric, ideal, ORIENTATION, 5, binding_reasons=(None, None), window_law=WINDOW_LAW_NARROW_BAND,
    )
    voronoi = certify_adaptive_density_fan(metric, ideal, ORIENTATION, 5, binding_reasons=(None, None))
    assert band != voronoi
    posing_as_voronoi = replace(band, proven_predicates=ADAPTIVE_FAN_PROVEN_PREDICATES)
    posing_as_band = replace(voronoi, proven_predicates=ADAPTIVE_FAN_NARROW_BAND_PROVEN_PREDICATES)
    for forged in (posing_as_voronoi, posing_as_band):
        with pytest.raises(AdaptiveDensityFanInvalid):
            verify_adaptive_density_fan(metric, ideal, ORIENTATION, forged)
    unknown = replace(band, proven_predicates=frozenset({"EVERYTHING_IS_FINE"}))
    with pytest.raises(AdaptiveDensityFanInvalid):
        verify_adaptive_density_fan(metric, ideal, ORIENTATION, unknown)


# --------------------------------------------------------------------------
# 2. Продукт: честные углы поля получают веер, близкий к равному шагу
# --------------------------------------------------------------------------


_DENSITIES = {
    1: (kernel.MaxSubturnValueId.LINEAR_REFLEX_DENSITY_1_V1, kernel.ExactAngleSymbol.PI_OVER_3),
    2: (kernel.MaxSubturnValueId.LINEAR_REFLEX_DENSITY_2_V1, kernel.ExactAngleSymbol.PI_OVER_4),
    4: (kernel.MaxSubturnValueId.LINEAR_REFLEX_DENSITY_4_V1, kernel.ExactAngleSymbol.PI_OVER_6),
}


def _load(name: str, density: int):
    """Слепок поля; запрос d1 слепок не несёт — берётся запрос d4 с другой плотностью."""

    base = HONEST.parent / name
    snapshot = kernel.AnalysisSnapshotCodecV1.loads(
        (base / "analysis_snapshot.json").read_bytes()
    )
    path = base / f"decal_request_density{density}.json"
    if path.exists():
        return snapshot, kernel.DecalRequestCodecV1.loads(path.read_bytes())
    donor = kernel.DecalRequestCodecV1.loads(
        (base / "decal_request_density4.json").read_bytes()
    )
    value_id, symbol = _DENSITIES[density]
    return snapshot, replace(
        donor,
        max_subturn_value_id=value_id,
        max_subturn_exact_value=kernel.ExactAngleV1(symbol),
    )


def _specs(compilation):
    return sorted(
        (item for item in compilation.envelope_specs if isinstance(item, AngularEnvelopeSpec)),
        key=lambda item: item.envelope_spec_id.value,
    )


def _fan_degrees(context, spec) -> list[float]:
    *_, normals = angular_support_data(context, spec)
    covectors = _covectors(context.metric, normals)

    def angle(left, right):
        cosine = _dual_dot(context.metric, left, right) / sp.sqrt(
            _dual_dot(context.metric, left, left) * _dual_dot(context.metric, right, right)
        )
        return math.degrees(math.acos(float(sp.N(cosine, 30))))

    return [angle(left, right) for left, right in zip(covectors, covectors[1:])]


@pytest.mark.parametrize("density", (1, 2, 4))
def test_an_honest_near_right_corner_is_bound_near_the_equal_step(density):
    """90.56 градуса — честное число, не канон: лучи привязаны в полосе вокруг равного шага."""

    snapshot, request = _load("building004_patch0", density)
    result = compile_reference_envelopes(snapshot, request)
    assert result.outcome is ReferenceOutcome.EXACT, result.diagnostics
    compilation = result.compilation
    assert not [
        item
        for item in compilation.diagnostics
        if item.outcome is ReferenceOutcome.ADAPTIVE_FAN_NARROW_BAND_NOT_APPLIED
    ]
    frame, diagnostics = validate_reference_geometry_payload(
        snapshot, compilation.plan_key.patch_domain_id, density_bounded=True
    )
    assert frame is not None, diagnostics
    context = GeometryContext.build(compilation, frame)
    seal_angular_support_cache(context)
    checked = 0
    for spec in _specs(compilation):
        authority = getattr(spec, "direction_fan_authority", None)
        if type(authority) is not AdaptiveMinimalRationalFanAuthorityV2:
            continue
        checked += 1
        assert authority_window_law(authority) == WINDOW_LAW_NARROW_BAND
        steps = _fan_degrees(context, spec)
        equal = sum(steps) / len(steps)
        angles = [sum(steps[: index + 1]) for index in range(len(steps) - 1)]
        for index, angle in enumerate(angles, start=1):
            assert abs(angle - index * equal) <= OMEGA_DEGREES + 1e-6, (steps, equal)
    assert checked, "the honest corners carry a bound fan"


# --------------------------------------------------------------------------
# 3. Отказ полосы назван, и веер ищет прежнее окно Вороного
# --------------------------------------------------------------------------


def test_a_band_that_cannot_hold_a_ray_is_a_named_refusal_and_the_voronoi_window_decides(
    monkeypatch,
):
    snapshot, request = _load("building004_patch0", 2)
    original_band = binding_module.certify_adaptive_huber_density_direction_fan
    original_voronoi = binding_module.certify_huber_density_bindings_with_adaptive_fallback
    attempts = []

    def refuse_the_band(*args, window_law=None, **kwargs):
        attempts.append(window_law)
        if window_law == WINDOW_LAW_NARROW_BAND:
            raise DensityRationalAuthorityExhausted("DENSITY_RATIONAL_AUTHORITY_EXHAUSTED")
        return original_band(*args, window_law=window_law, **kwargs)

    def voronoi(*args, **kwargs):
        attempts.append(WINDOW_LAW_VORONOI)
        return original_voronoi(*args, **kwargs)

    monkeypatch.setattr(binding_module, "certify_adaptive_huber_density_direction_fan", refuse_the_band)
    monkeypatch.setattr(binding_module, "certify_huber_density_bindings_with_adaptive_fallback", voronoi)
    result = compile_reference_envelopes(snapshot, request)
    assert result.outcome is ReferenceOutcome.EXACT, result.diagnostics
    refusals = [
        item
        for item in result.compilation.diagnostics
        if item.outcome is ReferenceOutcome.ADAPTIVE_FAN_NARROW_BAND_NOT_APPLIED
    ]
    assert refusals
    assert all("DensityRationalAuthorityExhausted" in item.message for item in refusals)
    assert all(item.envelope_spec_id is not None for item in refusals)
    assert WINDOW_LAW_NARROW_BAND in attempts and WINDOW_LAW_VORONOI in attempts
    refused_ids = {item.envelope_spec_id for item in refusals}
    voronoi_bound = {
        spec.envelope_spec_id.value
        for spec in _specs(result.compilation)
        if type(getattr(spec, "direction_fan_authority", None))
        is AdaptiveMinimalRationalFanAuthorityV2
        and authority_window_law(spec.direction_fan_authority) == WINDOW_LAW_VORONOI
    }
    assert refused_ids <= voronoi_bound
    prepared = prepare_conveyor(snapshot, request)
    assert prepared.outcome.value == "EXACT"
    assert prepared.counter("CONVEYOR_FAN_NARROW_BAND_REFUSED") == len(refusals)


def test_the_product_default_is_the_band_and_the_old_window_is_one_constant_away(monkeypatch):
    assert compile_module.FAN_WINDOW_LAW == WINDOW_LAW_NARROW_BAND
    snapshot, request = _load("building004_patch0", 2)
    banded = compile_reference_envelopes(snapshot, request).compilation
    monkeypatch.setattr(compile_module, "FAN_WINDOW_LAW", WINDOW_LAW_VORONOI)
    voronoi = compile_reference_envelopes(snapshot, request).compilation
    laws_banded = {
        authority_window_law(spec.direction_fan_authority)
        for spec in _specs(banded)
        if type(getattr(spec, "direction_fan_authority", None))
        is AdaptiveMinimalRationalFanAuthorityV2
    }
    laws_voronoi = {
        authority_window_law(spec.direction_fan_authority)
        for spec in _specs(voronoi)
        if type(getattr(spec, "direction_fan_authority", None))
        is AdaptiveMinimalRationalFanAuthorityV2
    }
    assert laws_banded == {WINDOW_LAW_NARROW_BAND}
    assert laws_voronoi <= {WINDOW_LAW_VORONOI}
    assert banded != voronoi


def test_no_band_refusal_is_reported_where_the_band_holds():
    for density in (1, 2):
        snapshot, request = _load("building004_patch0", density)
        result = compile_reference_envelopes(snapshot, request)
        assert not [
            item
            for item in result.compilation.diagnostics
            if item.outcome is ReferenceOutcome.ADAPTIVE_FAN_NARROW_BAND_NOT_APPLIED
        ]
        prepared = prepare_conveyor(snapshot, request)
        assert prepared.counter("CONVEYOR_FAN_NARROW_BAND_REFUSED") == 0
