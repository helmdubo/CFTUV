"""Бэкенд сборки меша декали в главном процессе (MESH_ASSEMBLY_NATIVE_V1): проверки собранного меша — нативно или эталоном ядра.

`CFTUV_MESH_ASSEMBLY_BACKEND` (`PYTHON` | `NATIVE`, по умолчанию `PYTHON`, пока весь `build_mesh_arrays` не перенесён) заказывает,
кто считает. Эталон — `cftuv_envelope.mesh_checks`; нативный путь — `cftuv_envelope.backend.mesh_assembly_compute` (единственный
импортёр `cftuv_native`). Отказ порта не тихий: он назван исходом ядра (`NATIVE_PORT_UNSUPPORTED`, `NATIVE_PORT_STALE`, ...),
сосчитан в `FALLBACKS` и один раз на причину напечатан; ответ тогда считает эталон, побитово тот же.
"""

from __future__ import annotations

import os

from cftuv_envelope import mesh_checks

from .envelope_production_weld import (
    COUNTER_FACES_OFF_PLANE_AFTER_OFFSET,
    COUNTER_MAX_OFF_PLANE_AFTER_OFFSET,
    NANOMETRES_PER_METRE,
)

MESH_ASSEMBLY_ENVIRONMENT = "CFTUV_MESH_ASSEMBLY_BACKEND"
MESH_ASSEMBLY_PYTHON = "PYTHON"
MESH_ASSEMBLY_NATIVE = "NATIVE"
DEFAULT_MESH_ASSEMBLY_BACKEND = MESH_ASSEMBLY_PYTHON
#: `{(исход, операция): [число, текст первой причины]}` откатов нативной сборки на эталон за процесс.
FALLBACKS: dict = {}


def mesh_assembly_backend() -> str:
    """Заказ среды процесса; неизвестное имя — `ValueError`."""

    text = os.environ.get(MESH_ASSEMBLY_ENVIRONMENT, "").strip().upper() or DEFAULT_MESH_ASSEMBLY_BACKEND
    if text not in (MESH_ASSEMBLY_PYTHON, MESH_ASSEMBLY_NATIVE):
        raise ValueError(f"unknown {MESH_ASSEMBLY_ENVIRONMENT} {text!r}: expected PYTHON or NATIVE")
    return text


def _native(operation: str, *arguments):
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

    found = _native("mesh_off_plane", positions, faces)
    worst, off = mesh_checks.off_plane_measure(positions, faces) if found is None else found
    return (
        (COUNTER_MAX_OFF_PLANE_AFTER_OFFSET, round(worst * NANOMETRES_PER_METRE)),
        (COUNTER_FACES_OFF_PLANE_AFTER_OFFSET, off),
    )


def cross_domain_seams(faces, uvs, face_domain) -> tuple:
    """Рёбра меша между гранями РАЗНЫХ доменов, где UV разрывен (`mesh_checks.cross_domain_seams`)."""

    found = _native("mesh_cross_seams", faces, uvs, face_domain)
    return mesh_checks.cross_domain_seams(faces, uvs, face_domain) if found is None else found


def half_edge_conflicts(faces) -> int:
    """Направленные рёбра двух граней (`mesh_checks.half_edge_conflicts`)."""

    found = _native("mesh_half_edges", faces)
    return mesh_checks.half_edge_conflicts(faces) if found is None else found


__all__ = (
    "DEFAULT_MESH_ASSEMBLY_BACKEND",
    "FALLBACKS",
    "MESH_ASSEMBLY_ENVIRONMENT",
    "cross_domain_seams",
    "half_edge_conflicts",
    "mesh_assembly_backend",
    "off_plane_after_offset",
)
