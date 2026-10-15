"""Отдельные низкоуровневые R3-записи: sign admission и Decimal-окно точного rational.

python tools/native_density_probes.py record --file DIR/synthetic.probes.xz
python tools/native_density_probes.py replay --file DIR/synthetic.probes.xz

Численно верный более точный знак НЕ равен названному отказу эталона.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import lzma
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
for _path in (ROOT / "kernel" / "src", ROOT / "tools"):
    if str(_path) not in sys.path:
        sys.path.insert(0, str(_path))

import sympy as sp  # noqa: E402

from cftuv_envelope.reference import angular, direction_binding, planar_types, symbolic_backend as symbolic  # noqa: E402
from native_density_corpus import Observation, applied_settings, settings  # noqa: E402
from native_density_synthetic import metric  # noqa: E402
from native_density_wire import CorpusWireError, Wire, json_key, require_keys  # noqa: E402

SCHEMA = "cftuv.native-density-probes.v1"
OPERATIONS = ("density_sign", "density_decimal_envelope")


def run_probe(wire: Wire, body: dict) -> dict:
    require_keys(body, ("operation", "expression", "metric", "settings"), "probe.input")
    if body["operation"] not in OPERATIONS:
        raise CorpusWireError("unsupported density probe operation")
    expression, owner_metric = wire.unexpression(body["expression"]), wire.unmetric(body["metric"])
    saved_counts, saved_symbolic = dict(symbolic.BACKEND_COUNTS), dict(planar_types.SYMBOLIC_FALLBACK_COUNTS)
    saved_disagreements = list(symbolic.DISAGREEMENTS)
    result = error = None
    try:
        with applied_settings(body["settings"]), Observation() as observed:
            try:
                if body["operation"] == "density_sign":
                    result = angular._density_exact_sign(expression, owner_metric)
                else:
                    result = direction_binding._density_decimal_envelope(expression)
            except Exception as caught:  # noqa: BLE001 - named refusal is the exact observable result
                error = caught
            return observed.outcome(result, error, [])
    finally:
        symbolic.BACKEND_COUNTS.clear()
        symbolic.BACKEND_COUNTS.update(saved_counts)
        planar_types.SYMBOLIC_FALLBACK_COUNTS.clear()
        planar_types.SYMBOLIC_FALLBACK_COUNTS.update(saved_symbolic)
        symbolic.DISAGREEMENTS[:] = saved_disagreements


def synthetic_document() -> dict:
    denominator = 1 << 256
    numerator = 213955882546153770510803628816683935098316969411928952863250352738335112326797
    nested = sp.sqrt(2 + sp.sqrt(2))
    cases = (
        ("nested_positive", "density_sign", nested),
        ("nested_negative", "density_sign", -nested),
        ("nested_tight_positive_refuses", "density_sign", nested - sp.Rational(numerator, denominator)),
        ("nested_tight_negative_refuses", "density_sign", nested - sp.Rational(numerator + 1, denominator)),
        ("q5_radical_residual", "density_sign", 8 * sp.Rational(4, 5) ** 2 - (3 + sp.sqrt(5))),
        ("rational_straw_1657_5120", "density_decimal_envelope", sp.Rational(1657, 5120)),
        ("rational_straw_3647_5120", "density_decimal_envelope", sp.Rational(3647, 5120)),
    )
    wire, rows = Wire(), []
    for name, operation, expression in cases:
        body = {"operation": operation, "expression": wire.expression(expression),
                "metric": wire.metric(metric()), "settings": settings()}
        outcome = run_probe(wire, body)
        rows.append({"name": name, "input": body, "outcome": outcome,
                     "id": hashlib.sha256(json_key([body, outcome]).encode("utf-8")).hexdigest()})
    return {"schema": SCHEMA, "expressions": wire.nodes, "records": rows}


def replay(document):
    require_keys(document, ("schema", "expressions", "records"), "probe.document")
    if document["schema"] != SCHEMA:
        raise CorpusWireError("unexpected probe schema")
    wire = Wire(document["expressions"])
    for index in range(len(wire.nodes)):
        wire.unexpression(index)
    failures = []
    for row in document["records"]:
        require_keys(row, ("name", "id", "input", "outcome"), "probe.record")
        if row["id"] != hashlib.sha256(json_key([row["input"], row["outcome"]]).encode("utf-8")).hexdigest():
            raise CorpusWireError("probe digest mismatch")
        actual = run_probe(wire, row["input"])
        if actual != row["outcome"]:
            failures.append({"name": row["name"], "expected": row["outcome"], "actual": actual})
    return {"records": len(document["records"]), "equal": len(document["records"]) - len(failures),
            "mismatch": len(failures), "first_mismatches": failures[:3]}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("command", choices=("record", "replay"))
    parser.add_argument("--file", type=Path, required=True)
    args = parser.parse_args(argv)
    if args.command == "record":
        document = synthetic_document()
        args.file.parent.mkdir(parents=True, exist_ok=True)
        args.file.write_bytes(lzma.compress(json_key(document).encode("ascii")))
    else:
        document = json.loads(lzma.decompress(args.file.read_bytes()).decode("ascii"))
    result = replay(document)
    print("NATIVE_DENSITY_PROBES " + json_key(result), flush=True)
    return int(bool(result["mismatch"]))


if __name__ == "__main__":
    raise SystemExit(main())
