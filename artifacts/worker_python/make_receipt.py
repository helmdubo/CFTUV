"""Расписка WORKER-PYTHON из выходов `button_probe.py` и `startup_probe.py`.

    python make_receipt.py <каталог с json> <куда RECEIPT.json> <корень базы> <корень ветки>

В каталоге лежат тройки `base_run<K>.json` (коммит f698e04, встроенный
интерпретатор, без настройки), `bundled_run<K>.json` (ветка, настройка пуста) и
`external_run<K>.json` (ветка, «Worker Python» = внешний CPython 3.13), плюс
`startup.json`. Каждый прогон — отдельный процесс Blender 4.5 на `building`,
QUEUE, Fan Density 2, 8 воркеров; шаги: `d2,d2,d2` (холодная и две тёплые кнопки),
четыре шага ползунка alpha, затем `d1`.

Равенство по каждому шагу каждого прогона — четыре пары: ветка/встроенный против
базы (встроенный путь не изменился), внешний против встроенного и внешний против
базы. Изъято из отпечатка только объявленное размещение: счётчики
`ENVELOPE_DOMAIN_POOL_*`, стадия `QUEUE_POOL_WALL` и хвост панели об интерпретаторе
и стене пула. Отрицательный контроль: искажённая часть отпечатка обязана быть
поймана тем же сравнением.
"""

from __future__ import annotations

import copy
import hashlib
import json
import statistics
import subprocess
import sys
from pathlib import Path

COLD = (0,)
WARM = (1, 2)
SLIDER = (3, 4, 5, 6)
SIDES = ("base", "bundled", "external")


def _runs(directory: Path, side: str) -> dict:
    return {
        path.stem.rsplit("run", 1)[1]: json.loads(path.read_text(encoding="utf-8"))
        for path in sorted(directory.glob(f"{side}_run*.json"))
    }


def _fingerprint(root: Path) -> str:
    digest = hashlib.sha256()
    for path in sorted((root / "cftuv").rglob("*.py")):
        if "__pycache__" in path.parts:
            continue
        digest.update(path.relative_to(root).as_posix().encode("utf-8"))
        digest.update(path.read_bytes().replace(b"\r\n", b"\n"))
    return digest.hexdigest()[:16]


def _head(root: Path) -> str:
    return subprocess.check_output(
        ("git", "-C", str(root), "rev-parse", "--short", "HEAD"),
        text=True,
        encoding="utf-8",
    ).strip()


def _same(left: dict, right: dict) -> tuple[bool, list]:
    differing = sorted(
        name
        for name in set(left["parts"]) | set(right["parts"])
        if left["parts"].get(name) != right["parts"].get(name)
    )
    same = (
        not differing
        and left["domain_outcomes"] == right["domain_outcomes"]
        and left["receipt_stage_counts"] == right["receipt_stage_counts"]
        and left["domains"] == right["domains"]
    )
    return same, differing


def _equality(left: dict, right: dict) -> dict:
    steps = []
    identical = left["failure"] is None and right["failure"] is None
    for one, other in zip(left["steps"], right["steps"]):
        same, differing = _same(one, other)
        identical = identical and same and one["label"] == other["label"]
        steps.append({"label": one["label"], "identical": same, "differing": differing})
    return {"identical": identical, "steps": steps}


def _median(values: list) -> float:
    return round(statistics.median(values), 3)


def _timing(runs: dict, indices) -> dict:
    samples = [
        run["steps"][index]["wall_seconds"] for run in runs.values() for index in indices
    ]
    return {"median_s": _median(samples), "min_s": min(samples), "samples_s": samples}


def _pool_wall(runs: dict, indices) -> float | None:
    samples = [
        run["steps"][index]["stage_totals"].get("QUEUE_POOL_WALL")
        for run in runs.values()
        for index in indices
    ]
    samples = [item for item in samples if item is not None]
    return _median(samples) if samples else None


def main() -> int:
    directory, out, base_root, branch_root = (
        Path(sys.argv[1]),
        Path(sys.argv[2]),
        Path(sys.argv[3]),
        Path(sys.argv[4]),
    )
    runs = {side: _runs(directory, side) for side in SIDES}
    common = sorted(set(runs["base"]) & set(runs["bundled"]) & set(runs["external"]))
    pairs = (
        ("bundled_vs_base", "bundled", "base"),
        ("external_vs_bundled", "external", "bundled"),
        ("external_vs_base", "external", "base"),
    )
    equality = {
        name: {
            run: _equality(runs[right][run], runs[left][run]) for run in common
        }
        for name, left, right in pairs
    }
    identical = all(
        item["identical"] for group in equality.values() for item in group.values()
    )

    # Отрицательный контроль: то же сравнение обязано поймать подмену части.
    probe = copy.deepcopy(runs["external"][common[0]])
    probe["steps"][1]["parts"]["sidecar"] = "0" * 16
    control_caught = not _equality(runs["bundled"][common[0]], probe)["identical"]

    timing = {}
    for side in SIDES:
        timing[side] = {
            "cold_press": _timing(runs[side], COLD),
            "warm_press": _timing(runs[side], WARM),
            "slider": _timing(runs[side], SLIDER),
            "pool_wall_cold_s": _pool_wall(runs[side], COLD),
            "pool_wall_warm_s": _pool_wall(runs[side], WARM),
        }
    speedup = {
        key: round(timing["bundled"][key]["median_s"] / timing["external"][key]["median_s"], 3)
        for key in ("cold_press", "warm_press", "slider")
    }
    labels = {
        side: sorted(
            {step["worker_python"] for run in runs[side].values() for step in run["steps"]},
            key=str,
        )
        for side in SIDES
    }
    startup = json.loads((directory / "startup.json").read_text(encoding="utf-8"))
    receipt = {
        # База — `git archive f698e04` в каталог без `.git`: коммит назван, а
        # дерево подтверждено отпечатком.
        "base": {
            "commit": "f698e04",
            "tree": _fingerprint(base_root),
            "note": "bundled interpreter, no Worker Python setting",
        },
        "branch": {"commit": _head(branch_root), "tree": _fingerprint(branch_root)},
        "mesh": "building, QUEUE, Fan Density 2, 8 workers, Blender 4.5.12",
        "steps": [step["label"] for step in runs["bundled"][common[0]]["steps"]],
        "runs": common,
        "identical": identical and control_caught,
        "negative_control_caught": control_caught,
        "placement_exempt": {
            "counters_prefix": "ENVELOPE_DOMAIN_POOL",
            "stages": ["QUEUE_POOL_WALL"],
            "panel_tail": (
                "| pool wall N ms on W workers (times above are per-domain sums)"
                " | worker Python X.Y.Z (bundled|external)"
                " | external Python rejected: <reason>"
            ),
        },
        "worker_python_in_panel": labels,
        "equality": equality,
        # Сами отпечатки частей: расхождение называло бы, ЧТО разошлось.
        "fingerprints": {
            side: {
                run: [
                    {
                        "step": step["label"],
                        "parts": step["parts"],
                        "panel": step["queue_timing_raw"][-110:],
                    }
                    for step in runs[side][run]["steps"]
                ]
                for run in common
            }
            for side in SIDES
        },
        "timing": timing,
        "speedup_bundled_over_external": speedup,
        "startup": {
            "rows": [
                {
                    "variant": row["variant"],
                    "median_s": row["median"],
                    "samples_s": row["ensure_started_seconds"],
                    "worker_sympy_dir": row["worker_sympy_dirs"],
                }
                for row in startup["rows"]
            ],
            "host_sympy_dir": startup["host_sympy_dir"],
            "kernel_py_files": startup["kernel_py_files"],
            "kernel_fingerprint_seconds_median": startup["kernel_fingerprint_seconds_median"],
            "describe_environment_seconds_median": startup["describe_environment_seconds_median"],
        },
    }
    out.write_text(
        json.dumps(receipt, ensure_ascii=False, indent=1, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    print("IDENTICAL" if receipt["identical"] else "DIFFERENT")
    return 0 if receipt["identical"] else 1


if __name__ == "__main__":
    sys.exit(main())
