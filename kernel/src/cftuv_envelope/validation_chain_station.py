"""Проверка плана станций цепей (`CHAIN_STATION_PLAN_V1`) в плане: структура и пересчёт по сырому снапшоту.

Живёт отдельно от `validation.py` по той же причине, что `validation_corner_treatment.py`: модуль стоит на потолке в
`tests/test_architecture.py`. Две ступени, как у обработки углов. Структурная видит один план: закон, допуск реестра, порядок и согласие
решения с причиной. Перекрёстная видит снапшот и пересчитывает КАЖДУЮ запись тем же законом, что и компиляция
(`_chain_station.plan_errors`): подделанное решение, пропущенная или лишняя цепь — именованный отказ `CHAIN_STATION_PLAN`.
Пересчёт проверяет и то, что решение не зависит от выбора цепей: закон читает только снапшот.
"""

from __future__ import annotations

from ._chain_station import plan_errors, structure_errors
from .validation_issues import ValidationCode, ValidationIssue, add_issue


def seam_neighbour_face_issues(snapshot) -> tuple[ValidationIssue, ...]:
    """Грани соседа шва снапшота (`seam_neighbour_faces`): имена не дублируют поверхность запроса, патч соседа вне патчей снапшота."""

    issues: list[ValidationIssue] = []
    own = {item.face_id for item in snapshot.surface_ir.source_faces}
    patches = {item.patch_id for item in snapshot.patches}
    for face in sorted(snapshot.seam_neighbour_faces, key=lambda item: item.face_id.value):
        path = ("seam_neighbour_faces", str(face.face_id))
        if face.face_id in own:
            add_issue(issues, ValidationCode.DUPLICATE_ID, path, "neighbour face repeats a face of the snapshot surface")
        if face.patch_id in patches:
            add_issue(issues, ValidationCode.CROSS_CONTRACT_MISMATCH, path, "neighbour face belongs to a patch of the snapshot: its surface is in surface_ir")
        if len(set(face.vertex_ids)) != len(face.vertex_ids):
            add_issue(issues, ValidationCode.SURFACE_TOPOLOGY, path, "neighbour face repeats a vertex")
    names = [item.face_id for item in snapshot.seam_neighbour_faces]
    if len(set(names)) != len(names):
        add_issue(issues, ValidationCode.DUPLICATE_ID, ("seam_neighbour_faces",), "two neighbour faces share a name")
    return tuple(issues)


def validate_plan_chain_stations(issues, plan) -> None:
    """Структура записей плана станций; снапшот не нужен."""

    for message in structure_errors(plan.chain_station_plans):
        add_issue(issues, ValidationCode.CHAIN_STATION_PLAN, ("chain_station_plans",), message)


def validate_plan_chain_stations_against_snapshot(issues, plan, snapshot, patch_domain_id, prefix) -> None:
    """Пересчёт плана станций по сырому снапшоту (`prefix` — путь плана в списке отказов)."""

    for message in plan_errors(snapshot, patch_domain_id, plan.chain_station_plans):
        add_issue(issues, ValidationCode.CHAIN_STATION_PLAN, (*prefix, "chain_station_plans"), message)
