"""Пишет реальные промежуточные iv-выражения при строгом повторе bind-корпуса."""

from __future__ import annotations

import argparse
from collections import Counter
from contextlib import contextmanager
import hashlib
import json
import lzma
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "kernel" / "src"))
sys.path.insert(0, str(ROOT / "tools"))
sys.set_int_max_str_digits(0)

from mpmath import iv
from cftuv_envelope import _density_policy as policy
from native_density_corpus import read_document, run_record
from native_density_wire import Wire, json_key


@contextmanager
def trace_intervals(wire, records, counts):
    original = policy.density_interval_enclosure
    saved = []

    def traced(expression, memo=None):
        precision = iv.prec
        try:
            result = original(expression, memo)
        except Exception as error:
            outcome = ["raised", type(error).__name__, str(error)]
            remember(expression, precision, outcome)
            raise
        outcome = ["ok", [[int(part) for part in bound] for bound in result._mpi_]]
        remember(expression, precision, outcome)
        return result

    def remember(expression, precision, outcome):
        row = [wire.expression(expression), precision, outcome]
        key = json_key(row)
        counts[key] += 1
        if counts[key] == 1:
            records.append(row)

    # Меняются только уже загруженные aliases этой конкретной функции.
    for name, module in list(sys.modules.items()):
        if name.startswith("cftuv_envelope") and module is not None:
            for attribute, value in list(vars(module).items()):
                if value is original:
                    saved.append((module, attribute, original))
                    setattr(module, attribute, traced)
    try:
        yield
    finally:
        for module, attribute, value in saved:
            setattr(module, attribute, value)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--file", action="append", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--limit", type=int)
    args = parser.parse_args()
    wire, records, counts = Wire(), [], Counter()
    sources, mismatches = [], []
    with trace_intervals(wire, records, counts):
        for path in args.file:
            document = read_document(path)
            rows = document["records"][:args.limit]
            for index, row in enumerate(rows):
                actual = run_record(document, row)
                if actual != row["outcome"]:
                    mismatches.append({"file": str(path), "index": index,
                                       "expected": row["outcome"], "actual": actual})
                if (index + 1) % 25 == 0:
                    print(json.dumps({"file": str(path), "done": index+1,
                                      "unique_intervals": len(records)}), flush=True)
            sources.append({"file": str(path), "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
                            "records": len(rows)})
    result = {"schema": "cftuv.density-interval-trace.v1", "sources": sources,
              "expressions": wire.nodes, "records": records,
              "occurrences": [counts[json_key(row)] for row in records],
              "mismatches": mismatches}
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_bytes(lzma.compress(json_key(result).encode("ascii")))
    summary = {"sources": sources, "nodes": len(wire.nodes), "unique_intervals": len(records),
               "calls": sum(counts.values()), "mismatches": len(mismatches),
               "sha256": hashlib.sha256(args.out.read_bytes()).hexdigest()}
    args.out.with_suffix(".receipt.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    print(json.dumps(summary), flush=True)
    return int(bool(mismatches))


if __name__ == "__main__":
    raise SystemExit(main())
