"""Сравнение двух выходов `button_probe.py`: всё, кроме секунд, обязано совпасть.

    python compare_probe.py <эталон.json> <ветка.json> [--receipt <куда.json>]

Код возврата 1, если хоть одна часть отпечатка любого шага разошлась; тогда
печатается КАКАЯ часть и какие счётчики/квитанции отличаются (из `detail`).
Секунды (`wall_seconds`, `stage_totals`) не сравниваются, а печатаются рядом:
это и есть то, что правка обязана менять.
"""

from __future__ import annotations

import json
import sys


def _load(path: str) -> dict:
    with open(path, encoding="utf-8") as handle:
        return json.load(handle)


def _diff_rows(left, right, limit=12):
    left_set = {tuple(map(str, row)) for row in left}
    right_set = {tuple(map(str, row)) for row in right}
    return (
        sorted(left_set - right_set)[:limit],
        sorted(right_set - left_set)[:limit],
    )


def _step_report(label, base, other, sequential) -> tuple[list[str], dict]:
    problems = []
    for name in sorted(set(base["parts"]) | set(other["parts"])):
        if base["parts"].get(name) != other["parts"].get(name):
            problems.append(f"{label}: part '{name}' differs")
    for key in ("domain_outcomes", "domains", "receipt_stage_counts"):
        if base[key] != other[key]:
            problems.append(f"{label}: {key} differs: {base[key]} vs {other[key]}")
    # Размещение объявлено (см. button_probe): счётчики пула и его стена — не
    # часть ответа. Но там, где пула нет вовсе (0 воркеров), их нет и у ветки.
    if sequential and (
        base["pool_counters"] != other["pool_counters"]
        or base["pool_stages"] != other["pool_stages"]
    ):
        problems.append(
            f"{label}: pool placement differs with no pool: "
            f"{base['pool_counters']} vs {other['pool_counters']}"
        )
    if problems:
        only_base, only_other = _diff_rows(
            base["detail"]["counters"], other["detail"]["counters"]
        )
        if only_base or only_other:
            problems.append(f"{label}: counters only in base {only_base}")
            problems.append(f"{label}: counters only in branch {only_other}")
        only_base, only_other = _diff_rows(
            base["detail"]["receipts"], other["detail"]["receipts"]
        )
        if only_base or only_other:
            problems.append(f"{label}: receipts only in base {only_base}")
            problems.append(f"{label}: receipts only in branch {only_other}")
    row = {
        "step": label,
        "wall_base": base["wall_seconds"],
        "wall_branch": other["wall_seconds"],
        "pool_counters_base": base["pool_counters"],
        "pool_counters_branch": other["pool_counters"],
        "pool_wall_base": base["stage_totals"].get("QUEUE_POOL_WALL"),
        "pool_wall_branch": other["stage_totals"].get("QUEUE_POOL_WALL"),
        "snapshot_export_base": base["stage_totals"].get("SNAPSHOT_EXPORT"),
        "snapshot_export_branch": other["stage_totals"].get("SNAPSHOT_EXPORT"),
        "patch_metric_export_base": base["stage_totals"].get("PATCH_METRIC_EXPORT"),
        "patch_metric_export_branch": other["stage_totals"].get("PATCH_METRIC_EXPORT"),
    }
    return problems, row


def main() -> int:
    base = _load(sys.argv[1])
    other = _load(sys.argv[2])
    problems = []
    rows = []
    sequential = int(base["workers"]) < 2 and int(other["workers"]) < 2
    if base.get("failure") or other.get("failure"):
        problems.append(f"probe failure: base={base.get('failure')} branch={other.get('failure')}")
    if len(base["steps"]) != len(other["steps"]):
        problems.append("step count differs")
    for left, right in zip(base["steps"], other["steps"]):
        if left["label"] != right["label"]:
            problems.append(f"step label differs: {left['label']} vs {right['label']}")
            continue
        step_problems, row = _step_report(
            left["label"], left, right, sequential
        )
        problems.extend(step_problems)
        rows.append(row)
    for row in rows:
        print(json.dumps(row))
    receipt = None
    if "--receipt" in sys.argv:
        receipt = sys.argv[sys.argv.index("--receipt") + 1]
        with open(receipt, "w", encoding="utf-8") as handle:
            json.dump(
                {
                    "base_root": base["root"],
                    "branch_root": other["root"],
                    "workers": [base["workers"], other["workers"]],
                    "identical": not problems,
                    "problems": problems,
                    "steps": rows,
                    "parts": {
                        step["label"] + f"#{index}": step["parts"]
                        for index, step in enumerate(other["steps"])
                    },
                },
                handle,
                ensure_ascii=False,
                indent=1,
                sort_keys=True,
            )
    if problems:
        print("DIFFERENT")
        for problem in problems:
            print(" ", problem)
        return 1
    print("IDENTICAL")
    return 0


if __name__ == "__main__":
    sys.exit(main())
