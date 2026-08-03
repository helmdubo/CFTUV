"""Общий пролог: stubs mathutils/bpy/bmesh из tests/conftest.py + пути."""

from __future__ import annotations

import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
# Корень дерева выводится из положения самого файла (artifacts/perf_prepare_diag/env.py),
# иначе харнесс воспроизводится только в том worktree, где его написали.
WT = HERE.parents[1]

for entry in (str(WT), str(WT / "kernel" / "src"), str(WT / "kernel" / "tests"), str(WT / "tests"), str(HERE)):
    if entry not in sys.path:
        sys.path.insert(0, entry)

# conftest.py ставит заглушки mathutils/bmesh/bpy прямо на импорте.
import conftest  # noqa: E402,F401

import mathutils  # noqa: E402

# Заглушка conftest покрывает ровно то, что трогают хост-тесты. Полевой путь
# `build_patch_graph` шире: ему нужны унарный минус, сравнение и хэш. Дополняем
# ЗАГЛУШКУ (в репозитории ничего не меняется).
_V = mathutils.Vector
if not hasattr(_V, "__neg__"):
    _V.__neg__ = lambda self: _V(tuple(-value for value in self._values))
if not hasattr(_V, "__eq__") or _V.__eq__ is object.__eq__:
    _V.__eq__ = lambda self, other: (
        isinstance(other, _V) and self._values == other._values
    )
    _V.__hash__ = lambda self: hash(self._values)
if not hasattr(_V, "to_tuple"):
    _V.to_tuple = lambda self, precision=-1: tuple(self._values)
if not hasattr(_V, "to_3d"):
    _V.to_3d = lambda self: _V(tuple(self._values[:3]))
if not hasattr(_V, "reflect"):
    _V.reflect = lambda self, other: self - other * (2.0 * self.dot(other))

SNAPSHOTS = WT / "artifacts" / "field_snapshots"
OUT = HERE
