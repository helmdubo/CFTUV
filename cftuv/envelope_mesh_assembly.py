"""Бэкенд сборки меша декали в главном процессе (MESH_ASSEMBLY_NATIVE_V1): проверки собранного меша — нативно или эталоном ядра.

`CFTUV_MESH_ASSEMBLY_BACKEND` (`PYTHON` | `NATIVE`, по умолчанию `NATIVE` — MESH_ASSEMBLY_NATIVE_DEFAULT_V1: сварка, проверки и отчёты
`build_mesh_arrays` перенесены; `PYTHON` — замороженный эталон и именованный откат) заказывает, кто считает. Эталон — `cftuv_envelope.mesh_checks`; нативный путь — `cftuv_envelope.backend.mesh_assembly_compute` (единственный
импортёр `cftuv_native`). Отказ порта не тихий: он назван исходом ядра (`NATIVE_PORT_UNSUPPORTED`, `NATIVE_PORT_STALE`, ...),
сосчитан в `FALLBACKS` и один раз на причину напечатан; ответ тогда считает эталон, побитово тот же.
"""

from __future__ import annotations

import os

from cftuv_envelope import mesh_checks


#: Числа плоскости граней после смещения: наибольшее отклонение вершины от плоскости своей грани (нм) и
#: число граней от четырёх вершин с отклонением больше нуля.
COUNTER_MAX_OFF_PLANE_AFTER_OFFSET = "ADAPTER_MAX_OFF_PLANE_AFTER_OFFSET_NANOMETRES"
COUNTER_FACES_OFF_PLANE_AFTER_OFFSET = "ADAPTER_FACES_OFF_PLANE_AFTER_OFFSET"
NANOMETRES_PER_METRE = 10**9
MESH_ASSEMBLY_ENVIRONMENT = "CFTUV_MESH_ASSEMBLY_BACKEND"
MESH_ASSEMBLY_PYTHON = "PYTHON"
MESH_ASSEMBLY_NATIVE = "NATIVE"
DEFAULT_MESH_ASSEMBLY_BACKEND = MESH_ASSEMBLY_NATIVE
#: `{(исход, операция): [число, текст первой причины]}` откатов нативной сборки на эталон за процесс.
FALLBACKS: dict = {}


def mesh_assembly_backend() -> str:
    """Заказ среды процесса; неизвестное имя — `ValueError`."""

    text = os.environ.get(MESH_ASSEMBLY_ENVIRONMENT, "").strip().upper() or DEFAULT_MESH_ASSEMBLY_BACKEND
    if text not in (MESH_ASSEMBLY_PYTHON, MESH_ASSEMBLY_NATIVE):
        raise ValueError(f"unknown {MESH_ASSEMBLY_ENVIRONMENT} {text!r}: expected PYTHON or NATIVE")
    return text


def native_answer(operation: str, *arguments):
    """Ответ порта либо `None` (заказан эталон или откат назван и записан)."""

    if mesh_assembly_backend() != MESH_ASSEMBLY_NATIVE:
        return None
    from cftuv_envelope.backend import mesh_assembly_compute

    answer, fallbacks = mesh_assembly_compute(operation, *arguments)
    for key, (count, detail) in fallbacks.items():
        slot = FALLBACKS.get(key)
        if slot is None:
            FALLBACKS[key] = [count, detail]
            print(f"[CFTUV][Production] MESH ASSEMBLY: {key[1]} computed in Python ({key[0]}: {detail})")
        else:
            slot[0] += count
    return answer


def off_plane_after_offset(positions, faces) -> tuple:
    """`((имя, число), ...)`: наибольшее отклонение граней от четырёх вершин от своей плоскости ПОСЛЕ смещения, нанометры, и число граней от нанометра."""

    found = native_answer("mesh_off_plane", positions, faces)
    worst, off = mesh_checks.off_plane_measure(positions, faces) if found is None else found
    return (
        (COUNTER_MAX_OFF_PLANE_AFTER_OFFSET, round(worst * NANOMETRES_PER_METRE)),
        (COUNTER_FACES_OFF_PLANE_AFTER_OFFSET, off),
    )


def mesh_reports(positions, faces, uvs, face_domain, face_owner, seams, chains) -> tuple:
    """Отчёты собранного меша одним вызовом: `(дайджест массивов, отчёт шва по цепям, плоскость граней после смещения, полурёбер двух граней)`.

    Порт читает массивы один раз (`cftuv_native.mesh_reports`); эталон — `mesh_checks.arrays_digest`, `seam_report_numbers`, `off_plane_measure`, `half_edge_conflicts`.
    """

    from .envelope_production_weld import COUNTER_SEAM_CLIP_VERTICES, COUNTER_SEAM_T_JUNCTIONS

    found = native_answer("mesh_reports", positions, faces, uvs, face_domain, face_owner, seams, chains)
    if found is None:
        found = (
            mesh_checks.arrays_digest(positions, faces, uvs, face_domain, face_owner, seams),
            mesh_checks.seam_report_numbers(chains),
            mesh_checks.off_plane_measure(positions, faces),
            mesh_checks.half_edge_conflicts(faces),
        )
    digest, (junctions, clip_vertices), (worst, off), conflicts = found
    return (
        digest,
        ((COUNTER_SEAM_T_JUNCTIONS, junctions), (COUNTER_SEAM_CLIP_VERTICES, clip_vertices)),
        ((COUNTER_MAX_OFF_PLANE_AFTER_OFFSET, round(worst * NANOMETRES_PER_METRE)), (COUNTER_FACES_OFF_PLANE_AFTER_OFFSET, off)),
        conflicts,
    )


def cross_domain_seams(faces, uvs, face_domain) -> tuple:
    """Рёбра меша между гранями РАЗНЫХ доменов, где UV разрывен (`mesh_checks.cross_domain_seams`)."""

    found = native_answer("mesh_cross_seams", faces, uvs, face_domain)
    return mesh_checks.cross_domain_seams(faces, uvs, face_domain) if found is None else found


def half_edge_conflicts(faces) -> int:
    """Направленные рёбра двух граней (`mesh_checks.half_edge_conflicts`)."""

    found = native_answer("mesh_half_edges", faces)
    return mesh_checks.half_edge_conflicts(faces) if found is None else found


__all__ = (
    "COUNTER_FACES_OFF_PLANE_AFTER_OFFSET",
    "COUNTER_MAX_OFF_PLANE_AFTER_OFFSET",
    "NANOMETRES_PER_METRE",
    "DEFAULT_MESH_ASSEMBLY_BACKEND",
    "FALLBACKS",
    "MESH_ASSEMBLY_ENVIRONMENT",
    "cross_domain_seams",
    "half_edge_conflicts",
    "mesh_assembly_backend",
    "mesh_reports",
    "native_answer",
    "off_plane_after_offset",
)
