"""Корпус скелета (`build_skeleton`): где лежит, как находится под ядро и как читается. Общая часть выгрузки, генератора, швов и сверки.

Единица замены — вызов `wavefront.skeleton.build_skeleton(polygon, ...)` целиком (`native_corpus.OP_SKELETON`): полигон решётки и состояние цены на
входе, `SkeletonV1` либо исключение и состояние цены на выходе. Корпусов три, и они лежат РЯДОМ с корпусом покрытия и резки того же ядра:

    <корпус ядра>/skeleton/            полевой: холодные подготовки пяти мешей сцены, `build_skeleton` каждого домена (`native_skeleton_export.py`)
    <корпус ядра>/synthetic_skeleton/  синтетический: вызовы из тестов ядра, сгенерированные полигоны, производные записи с урезанным потолком
    <корпус ядра>/synthetic_skeleton/seams/   швы S0: вызовы функций слоя времён событий и закона кандидата (`native_skeleton_seams.py`)

Корпус привязан к ядру ОТПЕЧАТКОМ КОДА (`kernel_identity` индекса = `clip_memo.kernel_code_identity()` процесса), как и корпус покрытия и резки:
записи старого ядра описывают другой эталон. Каталог корпуса ядра называется по HEAD (`native_corpus.corpus_directory`), но ищется по отпечатку.
"""

from __future__ import annotations

import json
import os
from pathlib import Path

import native_corpus as nc

FIELD_DIR = "skeleton"
SYNTHETIC_DIR = "synthetic_skeleton"
SEAMS_DIR = "seams"
KINDS = {"field": FIELD_DIR, "synthetic": SYNTHETIC_DIR}
#: Классы встроенных исключений, которыми эталон падает НЕ по замыслу: дефект ядра (не именованный отказ). Именованные исключения ядра (`ExactCanonicalizationWorkBudgetExhausted`,
#: `ZeroDivisorTimeError`, ...) — подклассы со своим именем и сюда не входят. Запись с таким исключением воспроизводима (класс и текст записаны), но нативный порт её точно не повторит.
INTERNAL_ERRORS = frozenset({"TypeError", "AttributeError", "KeyError", "IndexError", "AssertionError", "RuntimeError", "ZeroDivisionError", "NameError", "UnboundLocalError", "RecursionError"})


def is_internal_error(exception) -> bool:
    """Записанное исключение `(класс, текст)` — внутренний дефект эталона, а не именованный отказ."""

    return bool(exception) and exception[0] in INTERNAL_ERRORS


def _base(base: str | Path | None) -> Path:
    return Path(base or os.environ.get(nc.CORPUS_ENVIRONMENT) or nc.DEFAULT_CORPUS_BASE)


def default_out(kind: str, base: str | Path | None = None) -> Path:
    """Каталог корпуса `kind` ЭТОГО ядра: рядом с его корпусом покрытия и резки, а при отсутствии того — под каталогом по HEAD."""

    parent = nc.matching_corpus(str(base) if base else None) or nc.corpus_directory(nc.git_head(), str(base) if base else None)
    return parent / KINDS[kind]


def matching(kind: str, base: str | Path | None = None) -> Path | None:
    """Каталог корпуса `kind`, чей индекс записан под отпечаток кода ядра процесса (новейший); `None` — такого нет."""

    identity = nc.clip_memo.kernel_code_identity()
    found = []
    for path in _base(base).glob(f"*/{KINDS[kind]}/index.json"):
        try:
            if json.loads(path.read_text(encoding="utf-8")).get("kernel_identity") == identity:
                found.append(path)
        except (OSError, ValueError):
            continue
    return max(found, key=lambda item: item.stat().st_mtime).parent if found else None


def describe_missing(kind: str, base: str | Path | None = None) -> str:
    """Причина пропуска сверки, когда `matching` ничего не нашёл: чьи корпуса лежат и чем их собрать."""

    present = []
    for path in sorted(_base(base).glob(f"*/{KINDS[kind]}/index.json")):
        try:
            present.append(f"{path.parent.parent.name}={json.loads(path.read_text(encoding='utf-8')).get('kernel_identity')}")
        except (OSError, ValueError):
            present.append(f"{path.parent.parent.name}=<индекс не читается>")
    tool = "tools/native_skeleton_export.py" if kind == "field" else "tools/native_skeleton_synthetic.py build"
    return f"нет корпуса скелета ({kind}) под ядро {nc.clip_memo.kernel_code_identity()} в {_base(base)} (лежат: {', '.join(present) or 'ничего'}): `{tool}`"


def load_index(root: Path) -> dict:
    return json.loads((Path(root) / "index.json").read_text(encoding="utf-8"))


def rows_of(root: Path, *, derived: bool | None = None, mesh: str | None = None, op: str = nc.OP_SKELETON) -> list:
    """Строки индекса записей `op`: `derived` — `None` все, `True` только производные, `False` только прочие."""

    found = []
    for row in load_index(root)["records"]:
        if row["op"] != op or (mesh is not None and row["mesh"] != mesh):
            continue
        if derived is not None and (row.get("derived") is not None) != derived:
            continue
        found.append(row)
    return found


def read(root: Path, row: dict):
    return nc.read_record(Path(root) / row["path"])


def inventory(root: Path) -> dict:
    """Опись корпуса: записи по мешам и исходам, производные, размер, секунды вызовов, число полигонов по размеру."""

    index = load_index(root)
    rows = [row for row in index["records"] if row["op"] == nc.OP_SKELETON]
    by_mesh: dict = {}
    for row in rows:
        entry = by_mesh.setdefault(row["mesh"], {"records": 0, "derived": 0, "bytes": 0, "seconds": 0.0, "outcomes": {}})
        entry["records"] += 1
        entry["derived"] += int(row.get("derived") is not None)
        entry["bytes"] += row["bytes"]
        if row.get("derived") is None:
            entry["seconds"] += row["seconds"]
        entry["outcomes"][row["outcome"]] = entry["outcomes"].get(row["outcome"], 0) + 1
    return {
        "records": len(rows),
        "derived": sum(1 for row in rows if row.get("derived") is not None),
        "internal_errors_of_the_oracle": sum(1 for row in rows if is_internal_error(row["exception"])),
        "bytes": sum(row["bytes"] for row in rows),
        "kernel_identity": index.get("kernel_identity"),
        "python": index.get("python"),
        "by_mesh": by_mesh,
    }
