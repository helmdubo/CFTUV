"""Расписка свипа материализатора: три прогона -> один `RECEIPT.json`.

    python make_receipt.py <run_a.json> <run_b.json> <run_seq.json> <RECEIPT.json>

`run_a`, `run_b` — два прогона на 8 воркерах (детерминизм между прогонами),
`run_seq` — последовательный (`--workers 0`: 1 воркер против 8). В расписку
идут: исходы по плотностям, сводка секунд и счётчиков, результат сравнения
ответных полей (`sweep.ANSWER_KEYS`) и сами дайджесты по каждому домену.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import sweep  # noqa: E402


def _differences(left: dict, right: dict) -> list[str]:
    found = []
    for density in sorted(set(left["runs"]) & set(right["runs"])):
        a, b = left["runs"][density]["domains"], right["runs"][density]["domains"]
        if set(a) != set(b):
            found.append(f"d{density}: domain sets differ")
        for patch in sorted(set(a) & set(b), key=int):
            for key in sweep.ANSWER_KEYS:
                if a[patch].get(key) != b[patch].get(key):
                    found.append(f"d{density} patch{patch}: {key}")
    return found


def main() -> int:
    paths = [Path(item) for item in sys.argv[1:4]]
    out = Path(sys.argv[4])
    run_a, run_b, run_seq = (json.loads(item.read_text(encoding="utf-8")) for item in paths)
    receipt = {
        "schema": "materialize_sweep_receipt_v1",
        "sha": run_a["sha"],
        "alpha": run_a["alpha"],
        "python": run_a["python"],
        "cores": run_a["cores"],
        "determinism": {
            "run_a_vs_run_b_8_workers": _differences(run_a, run_b),
            "run_a_8_workers_vs_sequential": _differences(run_a, run_seq),
        },
        "densities": {},
    }
    for density, record in run_a["runs"].items():
        digests = {
            patch: {
                "content": row.get("content_digest", ""),
                "semantic": row.get("semantic_digest", ""),
                "outcome": f"{row['prepare_outcome']}/{row.get('coverage_outcome', '-')}/{row.get('materialization', '-')}",
            }
            for patch, row in record["domains"].items()
        }
        receipt["densities"][density] = {
            "summary_8_workers_a": record["summary"],
            "summary_8_workers_b": run_b["runs"][density]["summary"],
            "summary_sequential": run_seq["runs"][density]["summary"],
            "domains": digests,
        }
    out.write_text(
        json.dumps(receipt, ensure_ascii=False, sort_keys=True, indent=1),
        encoding="utf-8",
    )
    problems = sum(len(v) for v in receipt["determinism"].values())
    print("determinism problems:", problems)
    return 1 if problems else 0


if __name__ == "__main__":
    raise SystemExit(main())
