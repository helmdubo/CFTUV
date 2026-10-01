"""Собрать RECEIPT.json из дампов и отчётов проб (каталог прогона `--out`).

    python make_receipt.py <каталог с дампами> <RECEIPT.json>

Дампы `base*` сняты деревом 5392e01, `new*` — деревом с массовой записью.
Расписка хранит отпечатки и числа, не сами дампы (они мегабайты).
"""

from __future__ import annotations

import hashlib
import json
import statistics
import sys
from pathlib import Path


PAIRS = (
    ("base45", "new45", "Blender 4.5.12, building d2 (кнопка, QUEUE, 8 воркеров)"),
    ("base45s", "new45s", "Blender 4.5.12, смоки"),
    ("base43", "new43", "Blender 4.3.2 --factory-startup, смоки"),
)
BEFORE_REPORTS = ("base45_r1", "base45_r2", "base45i", "base45")
AFTER_REPORTS = ("new45_r1", "new45_r2", "new45")
# Только массовая запись, без памяти разбора координат (первый коммит).
BULK_ONLY_REPORTS = ("bulkonly45",)


def _digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _dump_counts(path: Path) -> dict:
    dump = json.loads(path.read_text(encoding="utf-8"))
    frames = [f for layer in dump["layers"] for f in layer["frames"]]
    return {
        "layers": len(dump["layers"]),
        "strokes": sum(len(f["strokes"]) for f in frames),
        "points": sum(
            s["point_count"] for f in frames for s in f["strokes"]
        ),
        "attributes": sorted(
            {name for f in frames for name in f.get("attributes", {})}
        ),
    }


def _equality(directory: Path) -> list[dict]:
    rows = []
    for before_tag, after_tag, environment in PAIRS:
        for before in sorted(directory.glob(f"{before_tag}_*_dump.json")):
            name = before.name[len(before_tag) + 1 : -len("_dump.json")]
            if "replay" in name:
                continue
            after = directory / f"{after_tag}_{name}_dump.json"
            if not after.exists():
                continue
            rows.append(
                {
                    "environment": environment,
                    "dump": name,
                    "before_sha256": _digest(before),
                    "after_sha256": _digest(after),
                    "byte_identical": before.read_bytes() == after.read_bytes(),
                    **_dump_counts(after),
                }
            )
    # building d2 без суффикса: файл называется `<tag>_dump.json`
    for before_tag, after_tag, environment in PAIRS[:1]:
        before = directory / f"{before_tag}_dump.json"
        after = directory / f"{after_tag}_dump.json"
        rows.insert(
            0,
            {
                "environment": environment,
                "dump": "building_d2_button",
                "before_sha256": _digest(before),
                "after_sha256": _digest(after),
                "byte_identical": before.read_bytes() == after.read_bytes(),
                **_dump_counts(after),
            },
        )
    return rows


def _timings(directory: Path, tags) -> list[dict]:
    rows = []
    for tag in tags:
        path = directory / f"{tag}_report.json"
        if not path.exists():
            continue
        report = json.loads(path.read_text(encoding="utf-8"))
        timings = report["profile"]["timings"]
        rows.append(
            {
                "run": tag,
                "gp_render_seconds": timings["GP_RENDER"],
                "button_wall_seconds": report["button_wall_seconds"],
                "queue_pool_wall_seconds": timings["QUEUE_POOL_WALL"],
            }
        )
    return rows


def _negative_control(directory: Path) -> dict:
    """Дамп обязан ловить сдвиг ОДНОЙ координаты на 2e-7 (порядка ULP float32)."""

    clean = directory / "base45s_queue_dump.json"
    mutated = directory / "new45s_control_mutated_dump.json"
    return {
        "clean_sha256": _digest(clean),
        "mutated_sha256": _digest(mutated),
        "detected": clean.read_bytes() != mutated.read_bytes(),
    }


def _median(rows, key):
    return round(statistics.median(row[key] for row in rows), 3)


def main(argv):
    directory, output = Path(argv[1]), Path(argv[2])
    before = _timings(directory, BEFORE_REPORTS)
    after = _timings(directory, AFTER_REPORTS)
    instrument = {}
    for label, tag in (
        ("before", "base45i"),
        ("bulk_only", "bulkonly45"),
        ("after", "new45"),
    ):
        path = directory / f"{tag}_report.json"
        report = json.loads(path.read_text(encoding="utf-8"))
        if report.get("instrument"):
            instrument[label] = report["instrument"][-1]
    receipt = {
        "schema": "cftuv.gp_render_bulk.receipt.v1",
        "base_commit": "5392e01",
        "scene": "E:/testscene.blend, mesh building, 458 seam edges, QUEUE, Fan Density 2",
        "equality": _equality(directory),
        "gp_render_before": before,
        "gp_render_bulk_only": _timings(directory, BULK_ONLY_REPORTS),
        "gp_render_after": after,
        "gp_render_median_seconds": {
            "before": _median(before, "gp_render_seconds"),
            "after": _median(after, "gp_render_seconds"),
        },
        "button_wall_median_seconds": {
            "before": _median(before, "button_wall_seconds"),
            "after": _median(after, "button_wall_seconds"),
        },
        "replay_inclusive_seconds": instrument,
        "negative_control": _negative_control(directory),
    }
    output.write_text(
        json.dumps(receipt, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    identical = all(row["byte_identical"] for row in receipt["equality"])
    print(f"receipt: {len(receipt['equality'])} dump pairs, all identical: {identical}")
    return 0 if identical else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv))
