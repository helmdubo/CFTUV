"""Сравнение прогона пула с последовательным эталоном (секунды исключены).

    python compare.py <baseline_sweep.json> <seq_fp.json> <pool_N.json> [...]

Два эталона: `baseline_sweep.json` — штатный `sweep.py` (исход, шесть статей
бюджета, SPENT, утечка, деталь) и `seq_fp.json` — тот же цикл в процессе с
отпечатками ответа (геометрия/счётчики/мета) и полным `prepared.counters`.
Любое расхождение — код возврата 1.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

SWEEP_KEYS_EXCLUDED = {"seconds"}
POOL_ONLY = {
    "host_export_seconds",
    "prepare_seconds",
    "coverage_seconds",
    "prepared_counters",
    "fp_geometry",
    "fp_meta",
    "fp_counters",
    "fp_host_counters",
    "repr_has_address",
    "patch_id",
    "pid",
    "t_start",
    "t_end",
    "rss_mb",
    "peak_wset_mb",
    "worker_startup",
    "pickle_probe",
    "error",
}


def load_rows(path):
    record = json.loads(Path(path).read_text(encoding="utf-8"))
    if "passes" in record:
        return {
            index: {row["patch_id"]: row for row in entry["rows"]}
            for index, entry in enumerate(record["passes"])
        }
    return {
        0: {
            int(key[5:]): dict(row, patch_id=int(key[5:]))
            for key, row in record["domains"].items()
        }
    }


def compare_to_sweep(base, rows):
    """Поля sweep.py (без секунд) побитово."""

    diffs = []
    if set(base) != set(rows):
        diffs.append(("domain-set", sorted(set(base) ^ set(rows))))
    for patch_id in sorted(set(base) & set(rows)):
        left, right = base[patch_id], rows[patch_id]
        for key in sorted(set(left) | set(right)):
            if key in SWEEP_KEYS_EXCLUDED or key in POOL_ONLY:
                continue
            if left.get(key, "<absent>") != right.get(key, "<absent>"):
                diffs.append(
                    (patch_id, key, left.get(key, "<absent>"),
                     right.get(key, "<absent>"))
                )
    return diffs


def compare_to_seq(seq, rows):
    """Отпечатки ответа и полный `prepared.counters` против эталона в процессе."""

    diffs = []
    keys = (
        "fp_geometry", "fp_meta", "fp_counters", "fp_host_counters",
        "prepared_counters", "repr_has_address",
    )
    for patch_id in sorted(set(seq) & set(rows)):
        for key in keys:
            if seq[patch_id].get(key, "<absent>") != rows[patch_id].get(
                key, "<absent>"
            ):
                diffs.append((patch_id, key))
    return diffs


def main():
    base_path, seq_path, *pool_paths = sys.argv[1:]
    base = load_rows(base_path)[0]
    seq = load_rows(seq_path)[0]
    verdicts = {}
    failed = False
    verdicts["seq_in_process_vs_sweep"] = compare_to_sweep(base, seq)
    failed |= bool(verdicts["seq_in_process_vs_sweep"])
    for path in pool_paths:
        for index, rows in load_rows(path).items():
            label = f"{Path(path).name}#pass{index}"
            sweep_diffs = compare_to_sweep(base, rows)
            seq_diffs = compare_to_seq(seq, rows)
            verdicts[label] = {
                "domains": len(rows),
                "vs_sweep_diffs": sweep_diffs,
                "vs_seq_fingerprint_diffs": seq_diffs,
            }
            failed |= bool(sweep_diffs or seq_diffs)
    print(json.dumps(verdicts, ensure_ascii=False, indent=1, default=str)[:6000])
    print("VERDICT", "MISMATCH" if failed else "IDENTICAL")
    sys.exit(1 if failed else 0)


if __name__ == "__main__":
    main()
