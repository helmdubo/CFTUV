"""Сборка RECEIPT.json среза SYMPY-OFF-HOT-PATH (шаги 1–2) из прогонов ворот.

Входы — файлы, которые пишут `artifacts/numeric_repr/gate.py run`, `artifacts/materialize_sweep/sweep.py run`
и `artifacts/sympy_off_hot_path/measure.py`, плюс таблица долей времени по местам вызова
(`audit_profile.json`, cProfile основы). Скрипт ничего не считает заново: он суммирует счётчики
символьного бэкенда по всем доменам и записывает, из каких файлов и какой командой они взяты.

    python artifacts/sympy_off_hot_path/make_receipt.py --out RECEIPT.json \
        --gate-shadow gate_shadow.json --gate-native gate_native.json \
        --sweep-shadow sweep_shadow.json --sweep-native sweep_native.json \
        --measure measure.json --audit-profile audit_profile.json --scene scene_exports.json --extra extra.json

`--gate-*` и `--sweep-*` принимают несколько файлов (плотности 1,2 и 4 идут отдельными прогонами ворот);
`--extra` — JSON чисел, которые не пишет ни один из инструментов (набор ядра под бэкендами, смоуки).
"""

from __future__ import annotations

import argparse
import collections
import json
import subprocess
from pathlib import Path

SCHEMA = "sympy_off_hot_path_receipt_v1"


def _walk(node, reports: list) -> None:
    if isinstance(node, dict):
        if "symbolic_backend_report" in node:
            reports.append(node["symbolic_backend_report"])
            rest = (value for key, value in node.items() if key != "symbolic_backend_report")
        elif "backend_counts" in node:
            reports.append(node)
            return
        else:
            rest = node.values()
        for value in rest:
            _walk(value, reports)
    elif isinstance(node, list):
        for value in node:
            _walk(value, reports)


def summarize(path: Path) -> dict:
    document = json.loads(path.read_text(encoding="utf-8"))
    reports: list = []
    _walk(document, reports)
    counts: collections.Counter = collections.Counter()
    disagreements = 0
    text_differences = 0
    for report in reports:
        counts.update(report.get("backend_counts", {}))
        disagreements += len(report.get("backend_disagreements", []))
        text_differences += len(report.get("backend_text_differences", []))
    checked = sum(v for k, v in counts.items() if k.endswith(".shadow_checked"))
    return {
        "file": path.name,
        "domain_rows": len(reports),
        "counts": dict(sorted(counts.items())),
        "shadow_checked_calls": checked,
        "disagreements": disagreements,
        "text_differences": text_differences,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--out", required=True)
    for name in ("gate-shadow", "gate-native", "sweep-shadow", "sweep-native"):
        parser.add_argument(f"--{name}", nargs="*", default=[])
    for name in ("measure", "audit-profile", "scene", "extra"):
        parser.add_argument(f"--{name}", default="")
    args = parser.parse_args()
    receipt: dict = {"schema": SCHEMA}
    try:
        receipt["sha"] = subprocess.run(
            ["git", "rev-parse", "--short", "HEAD"], capture_output=True, text=True, check=True
        ).stdout.strip()
    except (OSError, subprocess.CalledProcessError):
        receipt["sha"] = ""
    for key in ("gate_shadow", "gate_native", "sweep_shadow", "sweep_native"):
        value = getattr(args, key)
        if value:
            receipt[key] = [summarize(Path(item)) for item in value]
    if args.scene:
        receipt["scene_exports"] = json.loads(Path(args.scene).read_text(encoding="utf-8"))["summary"]
    if args.extra:
        receipt["recorded_by_hand"] = json.loads(Path(args.extra).read_text(encoding="utf-8"))
    if args.measure:
        receipt["measure"] = json.loads(Path(args.measure).read_text(encoding="utf-8"))
    if args.audit_profile:
        receipt["audit_profile"] = json.loads(Path(args.audit_profile).read_text(encoding="utf-8"))
    Path(args.out).write_text(json.dumps(receipt, indent=1, ensure_ascii=False), encoding="utf-8")
    print(json.dumps({k: v for k, v in receipt.items() if k in ("gate_shadow", "sweep_shadow")}, indent=1)[:3000])
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
