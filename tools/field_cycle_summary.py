"""Сводка полевого прогона: JSON ворот -> короткая таблица в консоль.

Ворота (`run_envelope_mr1_building_gate.py`) пишут полный JSON — он и есть
свидетельство. Но решение «стало лучше или хуже» принимается по нескольким
числам, и выуживать их из мегабайта глазами — ровно тот ручной шаг, на котором
цикл перестаёт быть циклом. Сводка печатает: домены и их исходы, Resolved N/M,
секунды по стадиям. Ничего не вычисляет заново — только читает файл ворот.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path


def summarise(path: Path) -> int:
    payload = json.loads(path.read_text(encoding="utf-8"))
    print(f"файл    : {path}")
    print(f"сцена   : {payload.get('source_file')}")
    print(f"меш     : {payload.get('source_object')}, статус {payload.get('status')}")
    unchanged = payload.get("source_mesh_unchanged")
    if unchanged is not None:
        print(f"меш не изменён прогоном: {unchanged}")

    exit_code = 0
    for run in payload.get("runs", ()):
        domains = run.get("domains", [])
        ready = [d for d in domains if d.get("stage") == "RAW_READY"]
        print()
        print(
            f"scope {run.get('scope')}: Resolved {len(ready)}/{len(domains)}, "
            f"topology {run.get('topology_elapsed_seconds', 0.0):.2f} c"
        )
        for domain in domains:
            runtime = domain.get("runtime") or {}
            performance = runtime.get("performance") or {}
            seconds = performance.get("raw_coverage_seconds")
            line = (
                f"  patch {domain.get('patch_id'):>3}  "
                f"{domain.get('stage'):<16} {domain.get('outcome')}"
            )
            if seconds is not None:
                line += f"  raw {seconds:.3f} c"
            print(line)
            if domain.get("stage") not in ("RAW_READY",):
                message = (domain.get("message") or "").splitlines()
                if message:
                    print(f"        {message[0][:120]}")
                exit_code = 1
    return exit_code


if __name__ == "__main__":
    target = Path(sys.argv[1]) if len(sys.argv) > 1 else None
    if target is None or not target.exists():
        raise SystemExit("нужен путь к JSON полевых ворот")
    sys.exit(summarise(target))
