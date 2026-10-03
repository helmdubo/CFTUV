"""Обработка вогнутого угла ДО закона счёта: компиляция и пересчёт записей по сырому снапшоту.

Сам закон («одна цепь» владельца, предел изгиба четверть оборота, причины) — в `_corner_treatment.py`:
его читает и проверяющий плана, которому `reference` недоступен. Здесь остаётся то,
что нужно только компиляции: выбор счёта угла под JOIN (`resolve_corner_selection`)
и пересчёт каждой записи компиляции при сборке `GeometryContext`
(`corner_treatment_errors`; расхождение — именованный отказ `CORNER_TREATMENT_INVALID`).

Каждый угол получает ровно одну запись `CornerTreatmentRecordV1` с причиной:
угол от порога, угол с интервалом поверх порога и излом, чья одна цепь владельца не
доказана, идут прежним законом счёта и названы, а не выбраны молча (AGENTS.md, п. 4).
Та же запись несётся в плане (`CompiledPatchEvaluationPlanV1.corner_treatments`) и
пересчитывается проверяющим плана по сырому снапшоту.
"""

from __future__ import annotations

from dataclasses import replace

from .._corner_treatment import (  # noqa: F401  (имена закона остаются здесь же по старому пути)
    CORNER_TREATMENT_LAW,
    JOIN_BEND_BOUND_OVER_PI,
    decide,
    recompute_record,
    treatment_record,
)
from ..contracts.analysis import CertifiedReflexAngleMeasureV1
from ..contracts.envelopes import CornerTreatmentV1, SelectionLaw
from .contracts import ReferenceOutcome


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
    return resolved, treatment_record(relation, sector, selection_id, measure, decision), None


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
        expected = recompute_record(
            relation, sector, selection_id, angle.measure_payload, uses_by_id, chains_by_id
        )
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
