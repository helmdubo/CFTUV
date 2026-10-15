"""Промежуточные знаки и реально достигнутый quadratic fallback bind-корпуса."""

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

from cftuv_envelope.reference import density_sign
from native_density_corpus import read_document, run_record
from native_density_wire import Wire, json_key


@contextmanager
def trace_signs(wire, records, counts):
    original = density_sign._density_exact_sign
    quadratic = density_sign.exact_quadratic_value
    saved, stack = [], []

    def field(expression):
        if stack:
            stack[-1].append(wire.expression(expression))
        return quadratic(expression)

    def traced(expression, metric=None):
        fallback = []
        stack.append(fallback)
        cached = metric is not None and expression in metric._density_exact_memo.signs
        try:
            try:
                result = original(expression, metric)
            except Exception as error:
                remember(expression, cached, fallback, ["raised", type(error).__name__, str(error)])
                raise
            remember(expression, cached, fallback, ["ok", int(result)])
            return result
        finally:
            stack.pop()

    def remember(expression, cached, fallback, outcome):
        row = [wire.expression(expression), cached, fallback, outcome]
        key = json_key(row)
        counts[key] += 1
        if counts[key] == 1:
            records.append(row)

    for name, module in list(sys.modules.items()):
        if name.startswith("cftuv_envelope") and module is not None:
            for attribute, value in list(vars(module).items()):
                if value is original:
                    saved.append((module, attribute, original))
                    setattr(module, attribute, traced)
    density_sign.exact_quadratic_value = field
    try:
        yield
    finally:
        density_sign.exact_quadratic_value = quadratic
        for module, attribute, value in saved:
            setattr(module, attribute, value)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--file", action="append", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    wire, records, counts = Wire(), [], Counter()
    sources, mismatches = [], []
    with trace_signs(wire, records, counts):
        for path in args.file:
            document = read_document(path)
            for index, row in enumerate(document["records"]):
                actual = run_record(document, row)
                if actual != row["outcome"]:
                    mismatches.append({"file": str(path), "index": index,
                                       "expected": row["outcome"], "actual": actual})
                if (index + 1) % 25 == 0:
                    print(json.dumps({"file": str(path), "done": index+1,
                                      "unique_signs": len(records)}), flush=True)
            sources.append({"file": str(path), "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
                            "records": len(document["records"])})
    result = {"schema": "cftuv.density-sign-trace.v1", "sources": sources,
              "expressions": wire.nodes, "records": records,
              "occurrences": [counts[json_key(row)] for row in records], "mismatches": mismatches}
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_bytes(lzma.compress(json_key(result).encode("ascii")))
    summary = {"sources": sources, "nodes": len(wire.nodes), "unique_signs": len(records),
               "calls": sum(counts.values()), "fallback_records": sum(bool(row[2]) for row in records),
               "mismatches": len(mismatches), "sha256": hashlib.sha256(args.out.read_bytes()).hexdigest()}
    args.out.with_suffix(".receipt.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    print(json.dumps(summary), flush=True)
    return int(bool(mismatches))


if __name__ == "__main__":
    raise SystemExit(main())
