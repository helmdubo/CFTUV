"""Лучи веера лифтованного канонического угла — точные рациональные повороты.

ПОЛЕВОЙ ДЕФЕКТ (`artifacts/fan_consistency/fan_steps.py`, замер 2026-10-03).
Прямой вогнутый угол при `q = 6` (d4) селектор видит как точный канон, а счёт
`H = 2` стоит РОВНО на `pi/6` с иррациональным лучом, поэтому счёт поднят до
`H = 3` (закон шума привязки, `evaluation_binding_noise.py`). Лучи поднятого
веера ставил адаптивный атлас: минимальная общая высота рационального веера в
окнах РАВНОУГОЛЬНОГО идеала `pi/8`, считанного на ШУМНОЙ вычислительной
геометрии. Сорок восемь конгруэнтных углов одной стены `2` получали десять
разных наборов шагов (до 14.8 градуса разброса внутри одного веера), то есть
«угол развёртывания и вектор развёртывания разные» у одинаковых оконных углов.

ЗАКОН `CANONICAL_FAN_RAYS_ON_CANONICAL_ANGLE_V1` (решение владельца:
конгруэнтные канонические углы получают ОДИН веер). Лучи не ищутся — они
ВЫЧИСЛЯЮТСЯ: луч ординала `j` — фиксированный рациональный поворот входящей
опоры по таблице `CANONICAL_ROTATION_TABLE` (`(1/2, 4, 6)`: пары `(12, 5)`,
`(1, 1)`, `(5, 12)`, то есть лучи на 22.62, 45 и 67.38 градуса и секторы
22.62, 22.38, 22.38, 22.62 при потолке 30), знак поворота — из ориентации угла
(как у равноугольного веера), а остаток шума остаётся в ПОСЛЕДНЕМ секторе.
Ряд палиндром: в обходе патча левый и правый угол окна идут в разном порядке,
и несимметричный ряд дал бы зеркальным углам разные веера. Рациональная
матрица поворота коммутирует с симметриями решётки, поэтому повёрнутые и
зеркальные конгруэнтные углы получают повёрнутые и зеркальные лучи ТОЧНО.

ГАРАНТИЯ НЕ ОСЛАБЛЕНА. Подшаг `<= pi/q` и порядок лучей проверяются точной
арифметикой на КАЖДОМ секторе вычислительной геометрии, включая последний с
шумом привязки; провал — именованный отказ, не допуск. Для `(1/2, 4, 6)`
запас по потолку 7.4 градуса при шуме привязки не больше 0.057 (объявленный
синус `1/1000`); замер поля — до 0.017 (`2.89e-4` рад).

ЭТО ЭВРИСТИКА, МЕНЯЮЩАЯ ОТВЕТ, поэтому она не молчит (п. 4 `AGENTS.md`): запись
`CanonicalRationalRotationFanAuthorityV1` — власть веера в самой спеке плана, а
отказ закона называет причину (`CanonicalFanRaysRefusalV1`) в диагностике ядра
и в счётчике `CONVEYOR_CANONICAL_FAN_RAYS_LAW_REFUSED`:

* нет записи таблицы для `(u, H + 1, q)` — прежний атлас;
* луч иррационален в карте (корень из определителя Грама не рационален) —
  прежний атлас;
* точная проверка подшага провалилась — прежний атлас.

НЕЗАВИСИМЫЙ ПРОВЕРЯЮЩИЙ пересчитывает всё по сырой геометрии: канонический факт
из интервала угла снапшота, запись таблицы, входящую опору вычислительной
геометрии, веер — и сверяет каждый записанный луч с вычисленным на точное
равенство направления (нулевой cross, положительный dot). Обратное тоже
проверяется: лифтованный канонический угол, к которому закон применим, не может
нести атлас.
"""

from __future__ import annotations

from dataclasses import dataclass
from fractions import Fraction

from .._density_policy import (
    CANONICAL_FAN_RAYS_PREDICATES,
    canonical_rotation_rays,
    huber_density_value_contract,
)
from ..contracts.envelopes import (
    AdaptiveDensityAngularEnvelopeSpecV2,
    CanonicalFanRaysLawV1,
    CanonicalRationalRotationFanAuthorityV1,
)
from ..numeric import ExactRatioV1
from .common import stable_id
from .contracts import (
    CanonicalFanRaysRefusalV1,
    ReferenceDiagnosticSeverity,
    ReferenceEvaluationDiagnosticV1,
    ReferenceOutcome,
)
from . import evaluation_binding_noise as noise
from .radical_rationality import radical_ratio_value

RAYS_LAW = CanonicalFanRaysLawV1.CANONICAL_FAN_RAYS_ON_CANONICAL_ANGLE_V1


@dataclass(frozen=True, slots=True)
class CanonicalFanRaysDecision:
    """Исход закона для одного лифтованного угла.

    `authority` есть — лучи поставлены по таблице. `refusal` есть — закон
    применим по факту, но не по условиям, и ответ решает прежний атлас.
    Обе пусты — закон молчит: угол не канонический либо привязки нет.
    """

    authority: CanonicalRationalRotationFanAuthorityV1 | None
    refusal: CanonicalFanRaysRefusalV1 | None


_SILENT = CanonicalFanRaysDecision(None, None)


def rotation_ideal(context, spec, count: int, rays):
    """`(входящая, лучи..., исходящая)`: лучи — повороты входящей опоры по таблице."""

    key = ("rotation-ideal", spec.envelope_spec_id.value, count, rays)
    cache = context.evaluation_noise_cache
    if key not in cache:
        from .angular import _incident_normal, _interpolated_normals

        relation, sector = noise._corner_ids(context, spec)
        incoming, _ = _incident_normal(
            context,
            sector.ordered_incident_chain_use_ids[0],
            relation.source_vertex_id,
        )
        outgoing, _ = _incident_normal(
            context,
            sector.ordered_incident_chain_use_ids[-1],
            relation.source_vertex_id,
        )
        cache[key] = _interpolated_normals(
            context.metric,
            incoming,
            outgoing,
            count,
            sector.turn_orientation,
            huber_density=True,
            rational_rotation=rays,
        )
    return cache[key]


def _primitive_covector(metric, unit_normal) -> tuple[int, int] | None:
    """Примитивный целый ковектор луча с ПОЛОЖИТЕЛЬНЫМ масштабом, либо `None`.

    Ковектор `G * n` единичной нормали `n` рационален по направлению ровно
    тогда, когда отношение его компонент рационально; знак масштаба — знак
    первой ненулевой компоненты. Ту же нормаль потребитель восстановит из
    вектора (`bound_unit_normal_from_vector`).
    """

    from .angular import _density_exact_sign

    x, y = metric.density_expressions(unit_normal)
    gram = metric.gram
    covector_x = gram[0][0] * x + gram[0][1] * y
    covector_y = gram[1][0] * x + gram[1][1] * y
    sign_x = _density_exact_sign(covector_x, metric)
    if sign_x == 0:
        sign_y = _density_exact_sign(covector_y, metric)
        return None if sign_y == 0 else (0, sign_y)
    ratio = radical_ratio_value(covector_y, covector_x)
    if ratio is None:
        return None
    return sign_x * ratio.denominator, sign_x * ratio.numerator


def _fan_holds(metric, ideal, q: int, orientation) -> bool:
    """Каждый сектор `<= pi/q` и каждый шаг строго по ориентации угла — ТОЧНО."""

    from .adaptive_density_fan import (
        _covectors,
        _expected_orientation,
        _oriented_cross,
        _sign,
        _subturn,
    )

    covectors = _covectors(metric, ideal)
    expected = _expected_orientation(orientation)
    return all(
        _subturn(metric, left, right, q)
        and _sign(_oriented_cross(metric, left, right), metric) == expected
        for left, right in zip(covectors, covectors[1:])
    )


def _authority(selection, fact, count, q, rays, vectors):
    canonical = fact.canonical
    return CanonicalRationalRotationFanAuthorityV1(
        authority_id=stable_id(
            "canonical-rational-rotation-fan-authority-v1",
            selection.certificate_id,
            canonical,
            count,
            q,
            rays,
            vectors,
        ),
        ray_law=RAYS_LAW,
        selection_certificate_id=selection.certificate_id,
        canonical_relation=fact.relation,
        canonical_reflex_excess_over_pi=ExactRatioV1(
            canonical.numerator, canonical.denominator
        ),
        hidden_edge_count=count,
        max_subturn_q=q,
        ray_rotation_pairs=rays,
        bound_primitive_integer_vectors=vectors,
        proven_predicates=CANONICAL_FAN_RAYS_PREDICATES,
    )


def canonical_fan_rays_decision(context, spec, selection) -> CanonicalFanRaysDecision:
    """Исход закона для спеки ЭФФЕКТИВНОГО счёта; кэшируется по `(спека, счёт)`.

    Читает только геометрию контекста и интервал угла снапшота: ни записи
    компиляции, ни прежние лучи спеки в решение не входят.
    """

    count = spec.resolved_hidden_edge_count
    key = ("rays-decision", spec.envelope_spec_id.value, count)
    cache = context.evaluation_noise_cache
    if key in cache:
        return cache[key]
    cache[key] = _decide(context, spec, selection, count)
    return cache[key]


def _decide(context, spec, selection, count: int) -> CanonicalFanRaysDecision:
    refusals = CanonicalFanRaysRefusalV1
    contract = huber_density_value_contract(selection.max_subturn_value_id)
    fact = (
        None
        if contract is None
        else noise.canonical_noise_fact(context, spec, selection)
    )
    if fact is None:
        return _SILENT
    q = contract[0]
    rays = canonical_rotation_rays(fact.canonical, count + 1, q)
    if rays is None:
        return CanonicalFanRaysDecision(
            None, refusals.NO_CANONICAL_ROTATION_TABLE_ENTRY
        )
    from .direction_binding import has_rational_density_support_direction

    ideal = rotation_ideal(context, spec, count, rays)
    metric = context.metric
    vectors = tuple(
        _primitive_covector(metric, ray)
        if has_rational_density_support_direction(metric, ray)
        else None
        for ray in ideal[1:-1]
    )
    if None in vectors:
        return CanonicalFanRaysDecision(
            None, refusals.CANONICAL_RAYS_IRRATIONAL_IN_CHART
        )
    _, sector = noise._corner_ids(context, spec)
    if not _fan_holds(metric, ideal, q, sector.turn_orientation):
        return CanonicalFanRaysDecision(
            None, refusals.CANONICAL_ROTATION_FAN_VIOLATES_SUBTURN_GUARANTEE
        )
    return CanonicalFanRaysDecision(
        _authority(selection, fact, count, q, rays, vectors), None
    )


def _selection_of(context, spec):
    return next(
        item
        for item in context.compilation.profile_selection_certificates
        if item.certificate_id == spec.selection_certificate_id
    )


def canonical_fan_rays_error(context, spec) -> str | None:
    """Пересчитать закон по сырой геометрии и назвать первое расхождение со спекой.

    Спека, не поднятая лифтом, законом не затрагивается. Лифтованная обязана
    нести ровно ту власть, что получилась бы: ни атласа там, где закон
    применим, ни власти закона там, где он молчит или отказал.
    """

    if (
        type(spec) is not AdaptiveDensityAngularEnvelopeSpecV2
        or spec.evaluation_subturn_count_lift is None
    ):
        return None
    authority = spec.direction_fan_authority
    carries = type(authority) is CanonicalRationalRotationFanAuthorityV1
    decision = canonical_fan_rays_decision(
        context, spec, _selection_of(context, spec)
    )
    if decision.authority is None:
        if not carries:
            return None
        return (
            "canonical rotation fan authority stands on an angle where the "
            "rays law does not apply"
            + ("" if decision.refusal is None else f": {decision.refusal.value}")
        )
    if not carries:
        return (
            "canonical fan rays law applies to this lifted angle, but its "
            "fan is not the canonical rotation fan"
        )
    expected = decision.authority
    if len(authority.bound_primitive_integer_vectors) != len(
        expected.bound_primitive_integer_vectors
    ):
        return "canonical rotation fan has another number of rays"
    error = _ray_direction_error(context, spec, authority, expected)
    if error is not None:
        return error
    if authority != expected:
        return "canonical rotation fan authority does not follow from the geometry"
    return None


def _ray_direction_error(context, spec, authority, expected) -> str | None:
    """Каждый записанный луч равен лучу поворота: нулевой cross и положительный dot.

    Сверка идёт по ВЕКТОРАМ записи против нормалей вычисленного веера — она не
    зависит от того, как выведен примитивный ковектор.
    """

    from .adaptive_density_fan import _covectors, _vector
    from .angular import _density_directions_agree

    count = authority.hidden_edge_count
    rays = canonical_rotation_rays(
        Fraction(
            authority.canonical_reflex_excess_over_pi.numerator,
            authority.canonical_reflex_excess_over_pi.denominator,
        ),
        count + 1,
        authority.max_subturn_q,
    )
    if rays is None or rays != authority.ray_rotation_pairs:
        return "canonical rotation fan names a rotation row that is not the table entry"
    ideal = rotation_ideal(context, spec, count, rays)
    covectors = _covectors(context.metric, ideal)
    for ordinal, vector in enumerate(
        authority.bound_primitive_integer_vectors, start=1
    ):
        if not _density_directions_agree(
            context.metric,
            covectors[ordinal],
            _vector(vector[0], vector[1], context.metric),
        ):
            return (
                "canonical rotation fan ray is not the exact ordinal "
                f"rotation: ordinal {ordinal}"
            )
    return None


def canonical_fan_rays_diagnostics(compilation, context, specs) -> tuple:
    """Именованный отказ закона там, где лифтованный канонический угол его не получил."""

    diagnostics = []
    for spec in sorted(
        (
            item
            for item in specs
            if type(item) is AdaptiveDensityAngularEnvelopeSpecV2
            and item.evaluation_subturn_count_lift is not None
        ),
        key=lambda item: item.envelope_spec_id.value,
    ):
        decision = canonical_fan_rays_decision(
            context, spec, _selection_of(context, spec)
        )
        if decision.refusal is None:
            continue
        diagnostics.append(
            ReferenceEvaluationDiagnosticV1(
                outcome=ReferenceOutcome.CANONICAL_FAN_RAYS_LAW_NOT_APPLIED,
                severity=ReferenceDiagnosticSeverity.INFO,
                message=(
                    f"{decision.refusal.value}: the rays of a lifted "
                    "canonical angle are found by the adaptive atlas on the "
                    "evaluation geometry, not set by the rotation table"
                ),
                envelope_spec_id=spec.envelope_spec_id.value,
            )
        )
    return tuple(diagnostics)


__all__ = (
    "RAYS_LAW",
    "CanonicalFanRaysDecision",
    "canonical_fan_rays_decision",
    "canonical_fan_rays_diagnostics",
    "canonical_fan_rays_error",
    "rotation_ideal",
)
