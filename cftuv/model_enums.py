"""Перечисления топологии хоста, которые читает выгрузка Envelope.

Живут отдельным модулем, а не в `model.py`, потому что `model.py` тянет
`mathutils`, а воркер пула доменов (`envelope_domain_pool`) — обычный
интерпретатор без Blender: выгрузка снапшота в воркере (`envelope_export_input`)
обязана импортировать `envelope_request_export`, а тому из всего `model.py`
нужны только эти три перечисления. `model.py` импортирует их отсюда же, поэтому
`cftuv.model.PatchType` и остальные — те же объекты, что и прежде.

Правило исполняется тестом `test_worker_export_modules_import_without_blender`.
"""

from __future__ import annotations

from enum import Enum


class PatchType(str, Enum):
    """Dispatch key for the patch UV strategy."""

    WALL = "WALL"
    FLOOR = "FLOOR"
    SLOPE = "SLOPE"


class LoopKind(str, Enum):
    """Kind of closed boundary loop."""

    OUTER = "OUTER"
    HOLE = "HOLE"


class ChainNeighborKind(str, Enum):
    """Topology class of a boundary chain neighbor."""

    PATCH = "PATCH"
    MESH_BORDER = "MESH_BORDER"
    SEAM_SELF = "SEAM_SELF"


__all__ = ("ChainNeighborKind", "LoopKind", "PatchType")
