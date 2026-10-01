"""Сравнение двух выходов `production_probe.py`: всё, кроме секунд и размещения.

    python compare_probe.py <эталон.json> <другой.json> [--steps cold,rebuild]

Сравнивается по шагам, общим у обоих файлов: отпечаток объекта (вершины, грани,
петли, швы, дайджест того, что лежит в Blender), исходы по доменам, содержательные
дайджесты каждого домена, числа материализатора, отказы и квитанция (дайджест
массивов, пропуски). Секунды, размещение (`placements`, счётчики пула) и счётчики
сборок (они зависят от того, был ли шаг холодным) не сравниваются — они и есть то,
что размещение имеет право менять. Код возврата 1 при любом расхождении.
"""

from __future__ import annotations

import json
import sys

ANSWER_RUN_KEYS = (
    "domains",
    "outcomes",
    "refused",
    "materialize_counters",
    "diagnostics",
    "chart_orientations",
    "content_digests",
)
ANSWER_RECEIPT_KEYS = (
    "arrays_digest",
    "mesh_digest",
    "mesh_name",
    "seam_edges",
    "seam_edges_requested",
    "skipped",
    "warnings",
    "domains",
)


def _load(path: str) -> dict:
    with open(path, encoding="utf-8") as handle:
        return json.load(handle)


def main() -> int:
    paths = [item for item in sys.argv[1:] if not item.startswith("--")]
    only = None
    if "--steps" in sys.argv:
        only = set(sys.argv[sys.argv.index("--steps") + 1].split(","))
        paths = [item for item in paths if item != sys.argv[sys.argv.index("--steps") + 1]]
    left, right = _load(paths[0]), _load(paths[1])
    problems = []
    if left.get("failure") or right.get("failure"):
        problems.append("a probe failed")
    right_steps = {item["label"]: item for item in right["steps"]}
    compared = 0
    for step in left["steps"]:
        label = step["label"]
        if label not in right_steps or (only is not None and label not in only):
            continue
        other = right_steps[label]
        compared += 1
        if step["object"] != other["object"]:
            problems.append(f"{label}: object stats differ")
        for key in ANSWER_RUN_KEYS:
            if step["run"].get(key) != other["run"].get(key):
                problems.append(f"{label}: run.{key} differs")
        for key in ANSWER_RECEIPT_KEYS:
            if step["run"]["receipt"].get(key) != other["run"]["receipt"].get(key):
                problems.append(f"{label}: receipt.{key} differs")
        if step["status"] != other["status"]:
            problems.append(f"{label}: status differs: {step['status']} / {other['status']}")
    for line in problems:
        print(line)
    print(
        f"compared {compared} steps: workers {left['workers']} vs {right['workers']}, "
        f"mesh {left['mesh']}, density {left['density']}"
    )
    print("IDENTICAL" if not problems else f"DIFFERENT ({len(problems)})")
    return 1 if problems else 0


if __name__ == "__main__":
    raise SystemExit(main())
