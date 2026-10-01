"""Пути диагностики: корень дерева выводится из положения файла.

Промежуточные данные (экспорт сцены, JSON-ы прогонов) лежат в scratchpad сессии,
каталог переопределяется переменной окружения FAN_SCRATCH. В репозиторий
попадают только скрипты и компактные итоговые таблицы.
"""
from __future__ import annotations

import os
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
_DEFAULT_SCRATCH = (
    "C:/Users/helmd/AppData/Local/Temp/claude/"
    "E--GITHUB-CFTUV--claude-worktrees-near-planar-proof-gates-4d3d00/"
    "e68b76b5-638e-42b9-8b74-79a953e1b36c/scratchpad"
)
SCRATCH = Path(os.environ.get("FAN_SCRATCH", _DEFAULT_SCRATCH))
EXPORT = SCRATCH / "export"
RESULTS = SCRATCH / "results"
if not RESULTS.exists():
    # чистый клон: перечни вееров, снятые на сцене владельца, лежат рядом со скриптами
    RESULTS = HERE / "data"

#: Меш -> каталог экспорта (имя меша с точкой заменяется на подчёркивание).
MESHES = ("2", "building.002", "building", "building.004", "2.001")


def mesh_dir(mesh: str) -> Path:
    return EXPORT / mesh.replace(".", "_")


def add_kernel_paths() -> None:
    """Ядро и хост берутся из ЭТОГО дерева."""

    for entry in (str(ROOT / "kernel" / "src"), str(ROOT)):
        if entry in sys.path:
            sys.path.remove(entry)
        sys.path.insert(0, entry)
