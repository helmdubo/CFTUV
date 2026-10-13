"""Инвентарь записанных Density DAG с явным охватом каждого источника."""

from __future__ import annotations

import argparse
from collections import Counter
import hashlib
import json
import lzma
from pathlib import Path
import sys

import sympy as sp

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "kernel" / "src"))
sys.set_int_max_str_digits(0)

from native_density_wire import Wire  # noqa: E402
from cftuv_envelope.reference.planar_types import exact_quadratic_value  # noqa: E402
from cftuv_envelope.reference.radical_v2 import from_v1  # noqa: E402


def inspect(path):
    payload = path.read_bytes()
    document = json.loads(lzma.decompress(payload))
    nodes = document["expressions"]
    wire = Wire(nodes)
    tags = Counter(node[0] for node in nodes)
    negative_powers = []
    integer_exponents = []
    integer_bits = []
    for index, node in enumerate(nodes):
        if node[0] == "Q":
            integer_bits.extend((abs(node[1]).bit_length(), node[2].bit_length()))
        if node[0] != "Pow":
            continue
        exponent = nodes[node[2]]
        if exponent[0] != "Q":
            continue
        numerator, denominator = exponent[1:]
        if denominator == 1:
            integer_exponents.append(numerator)
        if numerator >= 0:
            continue
        item = {"node": index, "exponent": [numerator, denominator]}
        # Ограничение инструмента явно записывается; это не отказ продукта.
        if denominator not in (1, 2) or abs(numerator) > 64:
            item["status"] = "NOT_INSPECTED_UNSUPPORTED_OR_LARGE_POWER"
        else:
            try:
                expression = wire.unexpression(node[1])
                if denominator == 2:
                    expression = sp.Pow(expression, sp.Rational(1, 2), evaluate=False)
                base = exact_quadratic_value(expression)
                powered = base
                for _ in range(abs(numerator)-1):
                    powered *= base
                converted, _ = from_v1(powered)
                item.update(status="VALUE_CONVERTED", inverse_classes=len(converted.terms))
            except (ValueError, ArithmeticError, TypeError) as exc:
                item.update(status="OLD_FIELD_REJECTED_OR_CONVERSION_FAILED", exception=type(exc).__name__)
        negative_powers.append(item)
    return {"file": str(path), "sha256": hashlib.sha256(payload).hexdigest(),
            "schema": document.get("schema"),
            "scope": {"cftuv.density-interval-trace.v1":
                      "Reached interval expressions during recorded bind replay; not all sign-only expressions",
                      "cftuv.density-sign-trace.v1":
                      "Reached sign expressions and actual expanded quadratic fallbacks during recorded bind replay"
                      }.get(document.get("schema"),
                            "Recorded input/memo DAG only; excludes newly constructed runtime expressions"),
            "records": len(document["records"]), "expression_nodes": len(nodes), "tags": dict(tags),
            "largest_integer_operand_bits": max(integer_bits, default=0),
            "largest_integer_exponent_bits": max((abs(n).bit_length() for n in integer_exponents), default=0),
            "integer_exponents": dict(Counter(map(str, integer_exponents))),
            "negative_powers": negative_powers,
            "inverse_over_three_classes": sum(item.get("inverse_classes", 0) > 3 for item in negative_powers)}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--file", type=Path, action="append", required=True)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    result = {"scope": "See per-file scope; unreached inputs are not covered",
              "files": [inspect(path) for path in args.file]}
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(result, indent=2), encoding="utf-8")
    print(json.dumps({"files": len(result["files"]),
                      "nodes": sum(item["expression_nodes"] for item in result["files"]),
                      "inverse_over_three_classes": sum(item["inverse_over_three_classes"] for item in result["files"])}), flush=True)


if __name__ == "__main__":
    main()
