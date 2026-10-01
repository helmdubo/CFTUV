"""Расписка WARM-COVERAGE-PARALLEL из выходов `button_probe.py`.

    python make_receipt.py <каталог с json> <куда RECEIPT.json> <корень базы> <корень ветки>

В каталоге лежат пары `base_w<N>_run<K>.json` / `branch_w<N>_run<K>.json`:

- `w8` (8 воркеров, `d2,d2,d2,d1,alpha:0.375,alpha:0.3,alpha:0.45,alpha:0.25,d1`):
  РАВЕНСТВО по каждому шагу каждого прогона (sidecar, домены очереди,
  счётчики вне `ENVELOPE_DOMAIN_POOL_*`, квитанции, свойства и штрихи GP, строки
  панели без хвоста пула, кэши сессии — канонические байты снапшотов, счёт
  сборок, ключи подготовок) и ВРЕМЯ: медиана настенных секунд шага по прогонам;
- `w0` (0 воркеров): равенство БЕЗ единого исключения, включая пулевые счётчики
  (их нет ни у базы, ни у ветки).

Что именно изъято из отпечатка, объявлено в `button_probe.py` (размещение), и
перечислено здесь же в поле `placement_exempt`.
"""

from __future__ import annotations

import hashlib
import json
import statistics
import subprocess
import sys
from pathlib import Path


def _runs(directory: Path, side: str, workers: int) -> dict:
    found = {}
    for path in sorted(directory.glob(f"{side}_w{workers}_run*.json")):
        found[path.stem.rsplit("run", 1)[1]] = json.loads(
            path.read_text(encoding="utf-8")
        )
    return found


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


def _equality(base: dict, branch: dict, sequential: bool) -> dict:
    steps = []
    identical = base["failure"] is None and branch["failure"] is None
    for left, right in zip(base["steps"], branch["steps"]):
        differing = sorted(
            name
            for name in set(left["parts"]) | set(right["parts"])
            if left["parts"].get(name) != right["parts"].get(name)
        )
        placement_ok = (
            not sequential
            or (
                left["pool_counters"] == right["pool_counters"]
                and left["pool_stages"] == right["pool_stages"]
            )
        )
        same = (
            not differing
            and left["domain_outcomes"] == right["domain_outcomes"]
            and left["receipt_stage_counts"] == right["receipt_stage_counts"]
            and left["domains"] == right["domains"]
            and placement_ok
        )
        identical = identical and same
        steps.append(
            {
                "label": left["label"],
                "identical": same,
                "differing_parts": differing,
                "pool_counters_base": left["pool_counters"],
                "pool_counters_branch": right["pool_counters"],
            }
        )
    return {"identical": identical, "steps": steps}


def _timing(base_runs: dict, branch_runs: dict) -> list:
    rows = []
    sample = next(iter(base_runs.values()))
    for index, step in enumerate(sample["steps"]):
        left = [run["steps"][index]["wall_seconds"] for run in base_runs.values()]
        right = [run["steps"][index]["wall_seconds"] for run in branch_runs.values()]
        rows.append(
            {
                "step": f"{index}:{step['label']}",
                "base_median_s": round(statistics.median(left), 3),
                "branch_median_s": round(statistics.median(right), 3),
                "base_runs_s": left,
                "branch_runs_s": right,
            }
        )
    return rows


def _stage_medians(runs: dict, index: int) -> dict:
    stages = {}
    for run in runs.values():
        for name, value in run["steps"][index]["stage_totals"].items():
            stages.setdefault(name, []).append(value)
    return {
        name: round(statistics.median(values), 3)
        for name, values in sorted(stages.items())
        if statistics.median(values) >= 0.05
    }


def main() -> int:
    directory, out, base_root, branch_root = (
        Path(sys.argv[1]),
        Path(sys.argv[2]),
        Path(sys.argv[3]),
        Path(sys.argv[4]),
    )
    receipt = {
        "base": {"commit": _head(base_root), "tree": _fingerprint(base_root)},
        "branch": {"commit": _head(branch_root), "tree": _fingerprint(branch_root)},
        "placement_exempt": {
            "counters_prefix": "ENVELOPE_DOMAIN_POOL",
            "stages": ["QUEUE_POOL_WALL"],
            "panel_tail": "| pool wall N ms on W workers (times above are per-domain sums)",
            "applies_when": "workers >= 2; at 0 workers nothing is exempt",
        },
    }
    ok = True
    for workers in (8, 0):
        base_runs = _runs(directory, "base", workers)
        branch_runs = _runs(directory, "branch", workers)
        if not base_runs or not branch_runs:
            continue
        key = f"w{workers}"
        equality = {
            run: _equality(base_runs[run], branch_runs[run], workers < 2)
            for run in sorted(set(base_runs) & set(branch_runs))
        }
        ok = ok and all(item["identical"] for item in equality.values())
        receipt[key] = {
            "runs": sorted(equality),
            "equality": equality,
            "timing": _timing(
                {run: base_runs[run] for run in equality},
                {run: branch_runs[run] for run in equality},
            ),
            "stage_medians_warm_press": {
                "base": _stage_medians(base_runs, 1),
                "branch": _stage_medians(branch_runs, 1),
            },
        }
    receipt["identical"] = ok
    out.write_text(
        json.dumps(receipt, ensure_ascii=False, indent=1, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    print("IDENTICAL" if ok else "DIFFERENT")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
