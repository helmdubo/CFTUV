"""Расписка HOST-EXPORT-PARALLEL из выходов `button_probe.py`.

    python make_receipt.py <каталог с json> <куда RECEIPT.json>

Читает в каталоге пары `base_w<N>_<tag>.json` / `branch_w<N>_<tag>.json`:

- `f1` (8 воркеров, `d2,d2,d1,alpha,d1`) и `f0` (0 воркеров, `d2,d2`) — РАВЕНСТВО:
  каждая часть отпечатка каждого шага (sidecar, домены очереди, счётчики,
  квитанции, свойства и штрихи GP, строки панели, кэши сессии — канонические
  байты снапшотов, счёт сборок, ключи подготовок) у ветки и базы совпала;
- `t1..t4` (8 воркеров, один холодный `d2`) — ВРЕМЯ: настенные секунды кнопки,
  стена пула и суммы стадий выгрузки до и после.

Отпечаток дерева (`cftuv/`) считается здесь, при сборке расписки: код ветки после
прогонов не менялся.
"""

from __future__ import annotations

import hashlib
import json
import statistics
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]


def _load(directory: Path, side: str, workers: int, tag: str):
    path = directory / f"{side}_w{workers}_{tag}.json"
    return json.loads(path.read_text(encoding="utf-8")) if path.exists() else None


def _fingerprint(root: Path) -> str:
    digest = hashlib.sha256()
    for path in sorted(root.rglob("*.py")):
        if "__pycache__" in path.parts:
            continue
        digest.update(path.relative_to(root).as_posix().encode("utf-8"))
        digest.update(path.read_bytes().replace(b"\r\n", b"\n"))
    return digest.hexdigest()[:16]


def _git(*arguments: str) -> str:
    return subprocess.check_output(
        ("git", "-C", str(ROOT), *arguments), text=True, encoding="utf-8"
    ).strip()


def _equality(base: dict, branch: dict) -> dict:
    steps = []
    identical = base["failure"] is None and branch["failure"] is None
    for left, right in zip(base["steps"], branch["steps"]):
        differing = sorted(
            name
            for name in set(left["parts"]) | set(right["parts"])
            if left["parts"].get(name) != right["parts"].get(name)
        )
        same = (
            not differing
            and left["domain_outcomes"] == right["domain_outcomes"]
            and left["receipt_stage_counts"] == right["receipt_stage_counts"]
            and left["pool_counters"] == right["pool_counters"]
        )
        identical = identical and same
        steps.append(
            {
                "step": left["label"],
                "identical": same,
                "differing_parts": differing,
                "parts": right["parts"],
                "receipt_stage_counts": right["receipt_stage_counts"],
                "pool_counters": right["pool_counters"],
                "wall_base": left["wall_seconds"],
                "wall_branch": right["wall_seconds"],
            }
        )
    return {"identical": identical, "steps": steps}


def _timing(base: dict, branch: dict) -> dict:
    left, right = base["steps"][0], branch["steps"][0]

    def total(step, name):
        return step["stage_totals"].get(name)

    return {
        "wall_base": left["wall_seconds"],
        "wall_branch": right["wall_seconds"],
        "pool_wall_base": total(left, "QUEUE_POOL_WALL"),
        "pool_wall_branch": total(right, "QUEUE_POOL_WALL"),
        "snapshot_export_base": total(left, "SNAPSHOT_EXPORT"),
        "snapshot_export_branch": total(right, "SNAPSHOT_EXPORT"),
        "patch_metric_export_base": total(left, "PATCH_METRIC_EXPORT"),
        "patch_metric_export_branch": total(right, "PATCH_METRIC_EXPORT"),
        "snapshot_validation_base": total(left, "SNAPSHOT_VALIDATION"),
        "snapshot_validation_branch": total(right, "SNAPSHOT_VALIDATION"),
        "frame_admission_base": total(left, "FRAME_ADMISSION"),
        "frame_admission_branch": total(right, "FRAME_ADMISSION"),
        "gp_render_base": total(left, "GP_RENDER"),
        "gp_render_branch": total(right, "GP_RENDER"),
    }


def main() -> None:
    directory = Path(sys.argv[1])
    output = Path(sys.argv[2])
    receipt: dict = {
        "schema": "host_export_parallel_receipt_v1",
        "base_commit": "5392e013200a640d2f978813011b135cc2513da0",
        "head_at_run": _git("rev-parse", "HEAD"),
        "branch_cftuv_fingerprint": _fingerprint(ROOT / "cftuv"),
        "kernel_fingerprint": _fingerprint(ROOT / "kernel" / "src" / "cftuv_envelope"),
        "scene": "E:\\testscene.blend",
        "mesh": "building",
        "engine": "QUEUE",
        "blender": "4.5.12 LTS, background, add-on loaded from the tree (not the installed copy)",
        "notes": [
            "EXPORT_FRAME_COST_SCALE 7 -> 5 (comment and constant) was edited while "
            "the batch ran; the scale multiplies every export task alike on a cold "
            "press and is not applied to snapshot tasks, so no run's task order depends on it.",
            "pool wall of the branch includes the export that moved into workers; "
            "the sum of PATCH_METRIC_EXPORT in the branch is worker seconds under 8-way load.",
        ],
    }
    for key, workers, tag in (("equality_w8", 8, "f1"), ("equality_w0", 0, "f0")):
        base = _load(directory, "base", workers, tag)
        branch = _load(directory, "branch", workers, tag)
        if base is not None and branch is not None:
            receipt[key] = _equality(base, branch)
    timings = []
    for tag in ("t1", "t2", "t3", "t4"):
        base = _load(directory, "base", 8, tag)
        branch = _load(directory, "branch", 8, tag)
        if base is not None and branch is not None:
            timings.append(_timing(base, branch))
    first = (
        _load(directory, "base", 8, "f1"),
        _load(directory, "branch", 8, "f1"),
    )
    if all(item is not None for item in first):
        timings.insert(0, _timing(*first))
    receipt["cold_press_d2_w8"] = timings
    for name in ("wall", "pool_wall", "snapshot_export", "patch_metric_export"):
        for side in ("base", "branch"):
            values = [item[f"{name}_{side}"] for item in timings if item.get(f"{name}_{side}") is not None]
            if values:
                receipt.setdefault("mean_seconds", {})[f"{name}_{side}"] = round(
                    statistics.mean(values), 3
                )
                receipt.setdefault("min_max_seconds", {})[f"{name}_{side}"] = [
                    round(min(values), 3),
                    round(max(values), 3),
                ]
    output.write_text(
        json.dumps(receipt, ensure_ascii=False, indent=1, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    print(json.dumps(receipt.get("mean_seconds", {}), indent=1))
    for key in ("equality_w8", "equality_w0"):
        if key in receipt:
            print(key, "identical" if receipt[key]["identical"] else "DIFFERENT")


if __name__ == "__main__":
    main()
