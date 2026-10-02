"""Проверка записей `CornerTreatmentRecordV1` ПЛАНА: структура и пересчёт по сырому снапшоту.

Живёт отдельно от `validation.py` по той же причине, что и `validation_metric.py`:
модуль стоит на своём потолке в `tests/test_architecture.py`.

Две ступени, как у восстановлений канонического угла. Структурная (`validate_plan_corner_treatments`)
видит один план: запись ссылается на сертификат селекции плана, закон сертификата и обработка
записи согласны, сертификат под `CORNER_JOIN_SOFT_BEND_V1` имеет честную запись JOIN (иначе `k = 0`
принималось бы на слово). Перекрёстная (`validate_plan_corner_treatments_against_snapshot`)
видит снапшот и пересчитывает КАЖДЫЙ угол плана заново тем же законом, что и компиляция
(`_corner_treatment.recompute_record`): подделанная запись, JOIN без доказанной цепи и
доказанный JOIN под прежним счётом — именованный отказ `CORNER_TREATMENT`.
"""

from __future__ import annotations

from ._corner_treatment import CORNER_TREATMENT_LAW, JOIN_THRESHOLD_OVER_PI, recompute_record
from .contracts.analysis import CertifiedReflexAngleMeasureV1
from .contracts.envelopes import CornerTreatmentReasonV1, CornerTreatmentV1, SelectionLaw
from .validation_issues import ValidationCode, add_issue


def _joined(certificate) -> bool:
    return certificate.selection_law is SelectionLaw.CORNER_JOIN_SOFT_BEND_V1


def validate_plan_corner_treatments(issues, plan) -> None:
    """Структура записей обработки угла плана; снапшот не нужен."""

    certificates = {item.certificate_id: item for item in plan.angular_profile_selection_certificates}
    seen = set()
    for record in sorted(plan.corner_treatments, key=lambda item: item.selection_certificate_id.value):
        path = ("corner_treatments", str(record.selection_certificate_id))
        if record.selection_certificate_id in seen:
            add_issue(issues, ValidationCode.DUPLICATE_ID, path, "two corner treatment records for one selection")
        seen.add(record.selection_certificate_id)
        certificate = certificates.get(record.selection_certificate_id)
        if certificate is None:
            add_issue(issues, ValidationCode.MISSING_REFERENCE, path, "treatment record names no selection certificate of the plan")
            continue
        join = record.treatment is CornerTreatmentV1.JOIN_CONTINUATION
        problems = (
            (record.treatment_law != CORNER_TREATMENT_LAW, "treatment law is not the declared one"),
            (record.corner_relation_id != certificate.corner_relation_id, "treatment record differs from its selection certificate corner"),
            (_joined(certificate) != join, "selection law and treatment disagree"),
            (_joined(certificate) and certificate.resolved_hidden_edge_count != 0, "JOIN corner must resolve k = 0"),
            (
                (record.threshold_over_pi.numerator, record.threshold_over_pi.denominator)
                != (JOIN_THRESHOLD_OVER_PI.numerator, JOIN_THRESHOLD_OVER_PI.denominator),
                "treatment record names another JOIN threshold",
            ),
            (
                join != (record.reason is CornerTreatmentReasonV1.SOFT_BEND_IN_ONE_SOURCE_CHAIN),
                "treatment reason does not belong to its treatment",
            ),
            (join and not record.shared_source_lineage_ids, "JOIN record carries no shared source chain"),
        )
        for failed, message in problems:
            if failed:
                add_issue(issues, ValidationCode.CORNER_TREATMENT, path, message)
    for certificate in plan.angular_profile_selection_certificates:
        if _joined(certificate) and certificate.certificate_id not in seen:
            add_issue(
                issues,
                ValidationCode.CORNER_TREATMENT,
                ("angular_profile_selection_certificates", str(certificate.certificate_id)),
                "CORNER_JOIN_SOFT_BEND_V1 selection has no corner treatment record",
            )


def validate_plan_corner_treatments_against_snapshot(issues, plan, snapshot, prefix) -> None:
    """Пересчёт обработки каждого угла плана по сырому снапшоту (`prefix` — путь плана в списке отказов)."""

    uses_by_id = {item.chain_use_id: item for item in snapshot.chain_uses}
    chains_by_id = {item.physical_chain_id: item for item in snapshot.physical_chains}
    sectors = {item.owner_sector_id: item for item in snapshot.angular_owner_sectors}
    relations = {item.corner_relation_id: item for item in snapshot.corner_relations}
    angles = {item.certificate_id: item for item in snapshot.reflex_angle_certificates}
    records = {item.selection_certificate_id: item for item in plan.corner_treatments}
    for certificate in plan.angular_profile_selection_certificates:
        relation = relations.get(certificate.corner_relation_id)
        sector = sectors.get(certificate.owner_sector_id)
        angle = angles.get(certificate.reflex_angle_certificate_id)
        path = (*prefix, "corner_treatments", str(certificate.certificate_id))
        if (
            relation is None
            or sector is None
            or angle is None
            or not isinstance(angle.measure_payload, CertifiedReflexAngleMeasureV1)
            or any(use not in uses_by_id for use in sector.ordered_incident_chain_use_ids)
            or any(uses_by_id[use].physical_chain_id not in chains_by_id for use in sector.ordered_incident_chain_use_ids)
        ):
            # Сырой факт не прочитан. Обработка, которой план НЕ заявляет (закон прежний, записи нет), пересчитывать
            # нечем и не нужно; заявленная (JOIN либо запись) без читаемого сертифицированного угла недоказуема.
            if _joined(certificate) or certificate.certificate_id in records:
                add_issue(issues, ValidationCode.CORNER_TREATMENT, path, "corner treatment is claimed over raw facts that cannot be read (no certified angle, relation, sector or chains in the snapshot)")
            continue
        expected = recompute_record(
            relation, sector, certificate.certificate_id, angle.measure_payload, uses_by_id, chains_by_id
        )
        record = records.get(certificate.certificate_id)
        if record is not None and record != expected:
            add_issue(issues, ValidationCode.CORNER_TREATMENT, path, "treatment record differs from the raw snapshot")
        if _joined(certificate) != (expected.treatment is CornerTreatmentV1.JOIN_CONTINUATION):
            add_issue(
                issues,
                ValidationCode.CORNER_TREATMENT,
                path,
                f"selection law {certificate.selection_law.value} contradicts the raw snapshot "
                f"({expected.treatment.value}, {expected.reason.value})",
            )
