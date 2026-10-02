"""Обработка вогнутого угла ДО закона счёта: JOIN мягкого излома одной цепи.

Решение владельца (2026-10-03): излом меньше 30° внутри ОДНОЙ цепи источника —
не веер, а продолжение полосы: `k = 0` (митра прямого скелета), `u`
непрерывна, шва нет. Порог 30° — тот же `CORNER_ANGLE_THRESHOLD_DEG` главного
UV-солвера: хост режет свои цепи на куски в точных изломах, а на углах от 30°
кончается сама цепь; «одна цепь» здесь — факт хоста (общая запись
`PhysicalChainV1.data_record_lineage` двух кусков), а порог сверяется с
СЕРТИФИЦИРОВАННЫМ интервалом δ/π ядра, нижней и верхней границей порознь.

Каждый угол получает ровно одну запись `CornerTreatmentRecordV1` с причиной:
угол от 30°, угол с интервалом поверх порога и излом между РАЗНЫМИ цепями идут
прежним законом счёта и названы, а не выбраны молча (AGENTS.md, п. 4). Без
общей записи хоста закон инертен: все прежние снапшоты и фикстуры дают прежний
ответ побитово.

`corner_treatment_errors` пересчитывает каждую запись по СЫРОМУ снапшоту при
каждой сборке `GeometryContext`; расхождение — именованный отказ
`CORNER_TREATMENT_INVALID`.
"""

from __future__ import annotations

from dataclasses import replace
from fractions import Fraction

from ..contracts.analysis import CertifiedReflexAngleMeasureV1
from ..contracts.envelopes import (
    CornerTreatmentReasonV1,
    CornerTreatmentRecordV1,
    CornerTreatmentV1,
    SelectionLaw,
)
from ..numeric import ExactRatioV1, IntervalEndpointKind
from .contracts import ReferenceOutcome

CORNER_TREATMENT_LAW = "CORNER_TREATMENT_V1"
#: 30° = π/6: порог главного UV-солвера, выраженный долей π рефлексного избытка.
JOIN_THRESHOLD_OVER_PI = Fraction(1, 6)


def shared_source_lineage(chain_a, chain_b) -> frozenset:
    """Общие записи хоста двух цепей: непусто — куски одной цепи источника."""

    return frozenset(chain_a.data_record_lineage) & frozenset(chain_b.data_record_lineage)


def softness(interval) -> CornerTreatmentReasonV1 | None:
    """`None` — δ < π/6 доказано; иначе причина, по которой JOIN не положен."""

    lower, upper = Fraction(interval.lower), Fraction(interval.upper)
    if upper < JOIN_THRESHOLD_OVER_PI or (
        upper == JOIN_THRESHOLD_OVER_PI
        and interval.upper_kind is IntervalEndpointKind.OPEN
    ):
        return None
    if lower >= JOIN_THRESHOLD_OVER_PI:
        return CornerTreatmentReasonV1.REFLEX_EXCESS_NOT_SOFT
    return CornerTreatmentReasonV1.REFLEX_EXCESS_INTERVAL_CONTAINS_THRESHOLD


def decide(sector, measure, uses_by_id, chains_by_id):
    """`(обработка, причина, общая линия)` одного угла по сырым фактам."""

    incoming, outgoing = (
        sector.ordered_incident_chain_use_ids[0],
        sector.ordered_incident_chain_use_ids[-1],
    )
    shared = shared_source_lineage(
        chains_by_id[uses_by_id[incoming].physical_chain_id],
        chains_by_id[uses_by_id[outgoing].physical_chain_id],
    )
    reason = softness(measure.reflex_excess_over_pi)
    if reason is not None:
        return CornerTreatmentV1.ANGULAR_PROFILE, reason, shared
    if not shared:
        return (
            CornerTreatmentV1.ANGULAR_PROFILE,
            CornerTreatmentReasonV1.SOURCE_CHAINS_DIFFER,
            shared,
        )
    return (
        CornerTreatmentV1.JOIN_CONTINUATION,
        CornerTreatmentReasonV1.SOFT_BEND_IN_ONE_SOURCE_CHAIN,
        shared,
    )


def _record(relation, sector, selection_id, measure, decision) -> CornerTreatmentRecordV1:
    treatment, reason, shared = decision
    return CornerTreatmentRecordV1(
        treatment_law=CORNER_TREATMENT_LAW,
        corner_relation_id=relation.corner_relation_id,
        selection_certificate_id=selection_id,
        incoming_chain_use_id=sector.ordered_incident_chain_use_ids[0],
        outgoing_chain_use_id=sector.ordered_incident_chain_use_ids[-1],
        treatment=treatment,
        reason=reason,
        threshold_over_pi=ExactRatioV1(
            JOIN_THRESHOLD_OVER_PI.numerator, JOIN_THRESHOLD_OVER_PI.denominator
        ),
        reflex_excess_over_pi=measure.reflex_excess_over_pi,
        shared_source_lineage_ids=shared,
    )


def resolve_corner_selection(
    request, relation, sector, angle_certificate, selection_id,
    uses_by_id, chains_by_id, resolve_profile,
):
    """`(выбор счёта, запись обработки, отказ|None)` одного `CornerRelation`.

    Закон счёта (`resolve_profile`) спрашивается ПЕРВЫМ, как раньше: угол без
    единственного профиля — прежний отказ, и JOIN его не прикрывает. Решение
    JOIN затем переписывает счёт на `k = 0` под своим законом.
    """

    if angle_certificate is None or not isinstance(
        angle_certificate.measure_payload, CertifiedReflexAngleMeasureV1
    ):
        return None, None, (
            ReferenceOutcome.ANGULAR_PROFILE_SELECTION_UNCERTAIN,
            f"CornerRelation {relation.corner_relation_id} lacks a certified numeric angle",
        )
    measure = angle_certificate.measure_payload
    resolved = resolve_profile(request, measure)
    if resolved is None:
        return None, None, (
            ReferenceOutcome.ANGULAR_PROFILE_SELECTION_UNCERTAIN,
            f"CornerRelation {relation.corner_relation_id} does not prove a unique angular profile",
        )
    decision = decide(sector, measure, uses_by_id, chains_by_id)
    if decision[0] is CornerTreatmentV1.JOIN_CONTINUATION:
        resolved = replace(
            resolved,
            hidden_count=0,
            selection_law=SelectionLaw.CORNER_JOIN_SOFT_BEND_V1,
            regression_fixture_id=None,
        )
    return resolved, _record(relation, sector, selection_id, measure, decision), None


def corner_treatment_errors(compilation) -> tuple[str, ...]:
    """Пересчёт каждой записи по сырому снапшоту; пусто — записи честны."""

    snapshot = compilation.analysis_snapshot
    uses_by_id = {item.chain_use_id: item for item in snapshot.chain_uses}
    chains_by_id = {item.physical_chain_id: item for item in snapshot.physical_chains}
    sectors = {item.owner_sector_id: item for item in snapshot.angular_owner_sectors}
    relations = {item.corner_relation_id: item for item in snapshot.corner_relations}
    angles = {item.certificate_id: item for item in snapshot.reflex_angle_certificates}
    selections = {
        item.certificate_id: item
        for item in compilation.profile_selection_certificates
    }
    errors = []
    by_selection = {}
    for record in compilation.corner_treatments:
        if record.selection_certificate_id in by_selection:
            errors.append(f"two treatment records for {record.selection_certificate_id}")
        by_selection[record.selection_certificate_id] = record
    for selection_id, selection in sorted(selections.items(), key=lambda item: item[0].value):
        record = by_selection.get(selection_id)
        joined = selection.selection_law is SelectionLaw.CORNER_JOIN_SOFT_BEND_V1
        if record is None:
            errors.append(f"selection {selection_id} has no corner treatment record")
            continue
        relation = relations.get(record.corner_relation_id)
        sector = sectors.get(selection.owner_sector_id)
        angle = angles.get(selection.reflex_angle_certificate_id)
        if (
            relation is None
            or sector is None
            or angle is None
            or relation.corner_relation_id != selection.corner_relation_id
            or not isinstance(angle.measure_payload, CertifiedReflexAngleMeasureV1)
        ):
            errors.append(f"treatment record {selection_id} names unknown raw facts")
            continue
        expected = _record(relation, sector, selection_id, angle.measure_payload,
                           decide(sector, angle.measure_payload, uses_by_id, chains_by_id))
        if record != expected:
            errors.append(f"treatment record {selection_id} differs from the raw snapshot")
        if joined != (record.treatment is CornerTreatmentV1.JOIN_CONTINUATION):
            errors.append(f"selection law and treatment disagree at {selection_id}")
        if joined and selection.resolved_hidden_edge_count != 0:
            errors.append(f"JOIN corner {selection_id} must resolve k = 0")
    for selection_id in by_selection:
        if selection_id not in selections:
            errors.append(f"treatment record {selection_id} has no selection certificate")
    return tuple(errors)
