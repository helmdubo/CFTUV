"""Read-only corpus comparison of recorded legacy shells and the A1 V2 candidate.

Recorded warm-bind calls are compared, never freshly recomputed by the old
evaluator. This is subset shell evidence, not full Density/R3 acceptance.
"""

from __future__ import annotations

import argparse
from collections import Counter
from contextlib import ExitStack
from dataclasses import asdict
from decimal import Decimal
from fractions import Fraction
import hashlib
from itertools import zip_longest
import json
import lzma
import os
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
for _path in (ROOT / "kernel" / "src", ROOT / "tools"):
    if str(_path) not in sys.path:
        sys.path.insert(0, str(_path))

from cftuv_envelope.reference.density_shell_v2 import (  # noqa: E402
    DENSITY_SHELL_CANON_V2_CANDIDATE,
    DensityShellOutcomeCode,
    DensityShellV2NotApplicable,
    density_shell_v2,
)
from density_decimal_projection import mpf_fraction, scaled_text  # noqa: E402
from native_density_wire import Wire  # noqa: E402


def _dump(value) -> str:
    return json.dumps(value, separators=(",", ":"), ensure_ascii=True)


def _pair(value: Fraction) -> list[int]:
    return [value.numerator, value.denominator]


def _observation(value: Decimal) -> dict:
    raw = value.as_tuple()
    return {"text": str(value), "value": _pair(Fraction(value)),
            "quantum": raw.exponent, "tuple": [raw.sign, list(raw.digits), raw.exponent]}


def _source_projection(intervals) -> tuple[tuple[Fraction, Fraction], list[int], list[str]]:
    if len(intervals) != 1:
        raise ValueError("expected exactly one recorded source enclosure")
    lower, upper = (mpf_fraction(raw) for raw in intervals[0])
    lo = lower.numerator * 10**27 // lower.denominator
    hi = -((-upper.numerator * 10**27) // upper.denominator)
    return (lower, upper), [lo, hi], [scaled_text(n, -27) for n in (lo, hi, hi-lo)]


def _count(counts: Counter, name: str, occurrences: int) -> None:
    counts[name + "_records"] += 1
    counts[name + "_calls"] += occurrences


def _write(streams, name: str, document) -> None:
    streams[name].write(_dump(document) + "\n")


def _accepted(key, result, source_bounds, source_units, source_decimals) -> dict:
    rational = result.value.as_rational()
    shell = result.shell
    decimals = [_observation(value) for value in (
        shell.lower, shell.upper, shell.absolute_error_bound)]
    units = [shell.lower_units, shell.upper_units]
    return {**key, "canon": result.canon,
            "outcomes": asdict(result.outcomes),
            "kind": "rational" if rational is not None else "irrational_quadratic",
            "value_rational": None if rational is None else _pair(rational),
            "v2_terms": result.value.terms,
            "v2_enclosure160": [_pair(value) for value in result.enclosure160],
            "v2_units1e27": units, "candidate_decimals": decimals,
            "diagnostic_v2_work": result.work.articles(),
            "mpi_changed": result.enclosure160 != source_bounds,
            "endpoints_changed": units != source_units,
            "width_changed": decimals[2]["value"] != source_decimals[2]["value"],
            "quantum_changed": [x["quantum"] for x in decimals] != [x["quantum"] for x in source_decimals],
            "tuple_changed": [x["tuple"] for x in decimals] != [x["tuple"] for x in source_decimals],
            "text_changed": [x["text"] for x in decimals] != [x["text"] for x in source_decimals]}


def _verify_recorded_straw(result, bounds, units) -> None:
    if DensityShellOutcomeCode.NON_DYADIC_GRID_STRAW not in result.outcomes.codes:
        return
    rational = result.outcomes.rational
    exact = Fraction(rational.numerator, rational.denominator)
    if not all(bound.denominator & (bound.denominator - 1) == 0 for bound in bounds):
        raise ValueError("recorded source endpoints are not finite binary rationals")
    if not bounds[0] < exact < bounds[1]:
        raise ValueError("recorded binary straw endpoints do not strictly enclose exact value")
    if units[1] - units[0] < result.outcomes.legacy_projected_min_width_units:
        raise ValueError("recorded source straw violates projected minimum width")


def _compare_record(key, expression, outcome, intervals, streams, counts, reasons, samples) -> None:
    source_decimals = [_observation(Decimal(text)) for text in outcome[1:]]
    bounds, units, projected = _source_projection(intervals)
    if projected != outcome[1:]:
        _count(counts, "source_projection_failure", key["occurrences"])
        _write(streams, "failures", {**key, "code": "SOURCE_PROJECTION_MISMATCH",
                                    "recorded": outcome[1:], "projected": projected})
    else:
        _count(counts, "source_projection_verified", key["occurrences"])
    try:
        result = density_shell_v2(expression)
        if not isinstance(result, DensityShellV2NotApplicable):
            _verify_recorded_straw(result, bounds, units)
    except Exception as error:
        # Ошибка не попадает в unsupported: отдельная провальная строка и exit 1.
        _count(counts, "candidate_failure", key["occurrences"])
        _write(streams, "failures", {**key, "code": "CANDIDATE_FAILURE",
                                    "type": type(error).__name__, "detail": str(error)})
        return
    if isinstance(result, DensityShellV2NotApplicable):
        _count(counts, "excluded", key["occurrences"])
        _count(reasons, result.code.value, key["occurrences"])
        _write(streams, "excluded", {**key, "canon": result.canon, "code": result.code.value,
                                    "detail": result.detail, "diagnostic_v2_work": result.work.articles()})
        return
    candidate = _accepted(key, result, bounds, units, source_decimals)
    _write(streams, "candidates", candidate)
    _count(counts, "accepted", key["occurrences"])
    _count(counts, "accepted_" + candidate["kind"], key["occurrences"])
    for name in ("mpi", "endpoints", "width", "quantum", "tuple", "text"):
        if candidate[name + "_changed"]:
            _count(counts, name + "_changed", key["occurrences"])
    if any(candidate[name + "_changed"] for name in ("endpoints", "width", "tuple", "text")):
        _count(counts, "changed", key["occurrences"])
        _count(counts, "changed_" + candidate["kind"], key["occurrences"])
        changed = {**candidate, "source_decimals": source_decimals,
                   "source_units1e27": units, "source_enclosure160": [_pair(x) for x in bounds]}
        _write(streams, "changed", changed)
        if len(samples) < 10:
            samples.append(changed)


def _inspect(path: Path, streams) -> dict:
    payload = path.read_bytes()
    document = json.loads(lzma.decompress(payload))
    if document["schema"] != "cftuv.density-decimal-trace.v2" or document["mismatches"]:
        raise ValueError("requires a mismatch-free density-decimal-trace.v2 corpus")
    if len(document["records"]) != len(document["occurrences"]):
        raise ValueError("record/occurrence lengths differ")
    wire = Wire(document["expressions"])
    counts, reasons, samples = Counter(), Counter(), []
    for index, (node, outcome, intervals) in enumerate(document["records"]):
        key = {"file": path.name, "record": index, "node": node,
               "occurrences": document["occurrences"][index]}
        _write(streams, "source-values", {**key, "outcome": outcome, "mpi": intervals,
                                         "decimals": ([_observation(Decimal(x)) for x in outcome[1:]]
                                                      if outcome[0] == "ok" else None)})
        _count(counts, "total", key["occurrences"])
        if outcome[0] != "ok":
            _count(counts, "source_raised_not_shell_input", key["occurrences"])
            _write(streams, "excluded", {**key, "code": "SOURCE_RAISED_NOT_SHELL_INPUT", "outcome": outcome})
            continue
        _compare_record(key, wire.unexpression(node), outcome, intervals,
                        streams, counts, reasons, samples)
    summary = {"file": str(path), "sha256": hashlib.sha256(payload).hexdigest(),
               "recorded_scope": document["scope"], "sources": document["sources"],
               "counts": dict(counts), "exclusions": dict(reasons), "changed_samples": samples}
    print(_dump({"file_complete": path.name, "counts": dict(counts), "exclusions": dict(reasons)}), flush=True)
    return summary


def _manifest(paths) -> list[dict]:
    return [{"path": str(path), "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
             "bytes": path.stat().st_size} for path in paths]


def _verify_pre_bundle_records(baseline: Path, current: Path) -> dict:
    differences, compared = [], 0
    with baseline.open(encoding="ascii") as old, current.open(encoding="ascii") as new:
        for index, (old_line, new_line) in enumerate(zip_longest(old, new)):
            if old_line is None or new_line is None:
                differences.append({"index": index, "reason": "candidate record count differs"})
                continue
            previous, updated = json.loads(old_line), json.loads(new_line)
            updated.pop("outcomes")
            if previous != updated:
                differences.append({"index": index, "file": previous["file"], "record": previous["record"],
                                    "fields": sorted(key for key in previous.keys() | updated.keys()
                                                     if previous.get(key) != updated.get(key))})
            compared += 1
    return {"scope": "Every pre-bundle candidate record field, including exact shell values/tuples/text and V2 work, unchanged after removing new outcomes only.",
            "baseline": _manifest([baseline])[0], "compared_records": compared, "differences": differences}


def _receipt(sources, args, started) -> dict:
    totals, exclusions = Counter(), Counter()
    for source in sources:
        totals.update(source["counts"])
        exclusions.update(source["exclusions"])
    failed = totals["candidate_failure_records"] or totals["source_projection_failure_records"]
    receipt = {"schema": "cftuv.density-shell-v2-diff.v1", "canon": DENSITY_SHELL_CANON_V2_CANDIDATE,
               "status": "FAILED" if failed else "COMPLETED", "process": {"pid": os.getpid()},
               "scope": "All recorded warm-bind Decimal calls; whole-AST grammar subset only. No full bind, turn, compile or R3 parity claim.",
               "method": "Raw SymPy AST -> direct RadicalSumV2 -> whole-value enclosure(160) -> integer floor/ceil -> exact quantum -27 Decimal tuples. No V1 reader, normalization or legacy evaluator fallback.",
               "price_scope": "V2 work is diagnostic only. DensityWork unchanged.",
               "decimal_quantum_policy": "Endpoint and width values, tuples, quantum and text compared separately; width derives from integer units.",
               "outcome_semantics": {
                   "DENSITY_DECIMAL_SHELL_V2_APPLIED": "Present on every accepted candidate.",
                   "DENSITY_DECIMAL_GRID_POINT_V2": "Whole canonical value is reduced rational n/d with d dividing 10**27; immutable metadata gives n, d and exact grid units.",
                   "DENSITY_DECIMAL_NON_DYADIC_GRID_STRAW_V2": "Grid point whose reduced denominator is not a power of two. Finite binary outward endpoints cannot equal this value, so floor/ceil projection has width at least 2 units, not necessarily exactly 2. A mathematical value fact, not an observed-width claim or old evaluator call.",
                   "recorded_straw_verification": "For each named straw in the trace, verify finite-binary endpoints, strict enclosure of the exact rational, and projected source width >=2 units.",
               },
               "sources": sources, "totals": dict(totals), "exclusions": dict(exclusions),
               "elapsed_seconds_diagnostic_only": time.monotonic() - started}
    if args.prior:
        prior = json.loads(args.prior.read_text(encoding="utf-8"))
        receipt["prior_probe_readonly"] = {"path": str(args.prior),
                                           "sha256": hashlib.sha256(args.prior.read_bytes()).hexdigest(),
                                           "totals": prior["totals"],
                                           "policy": "Historical observation only; no automatic expectation or acceptance gate."}
    return receipt


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--file", type=Path, action="append", required=True)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--prior", type=Path)
    parser.add_argument("--pre-bundle-candidates", type=Path)
    args = parser.parse_args(argv)
    sys.set_int_max_str_digits(0)
    started = time.monotonic()
    args.out.mkdir(parents=True, exist_ok=True)
    with ExitStack() as stack:
        streams = {name: stack.enter_context((args.out / (name + ".jsonl")).open("w", encoding="ascii"))
                   for name in ("source-values", "candidates", "changed", "excluded", "failures")}
        sources = [_inspect(path, streams) for path in args.file]
    receipt = _receipt(sources, args, started)
    if args.pre_bundle_candidates:
        parity = _verify_pre_bundle_records(args.pre_bundle_candidates, args.out / "candidates.jsonl")
        receipt["pre_bundle_candidate_parity"] = parity
        if parity["differences"]:
            receipt["status"] = "FAILED"
    receipt["source_files"] = _manifest([
        Path(__file__).resolve(), ROOT / "tools/native_density_wire.py", ROOT / "tools/density_decimal_projection.py",
        ROOT / "kernel/src/cftuv_envelope/reference/density_shell_v2.py",
        ROOT / "kernel/src/cftuv_envelope/reference/radical_v2.py",
    ])
    receipt["artifacts"] = _manifest(sorted(args.out.glob("*.jsonl")))
    receipt_path = args.out / "receipt.json"
    receipt_path.write_text(json.dumps(receipt, indent=2), encoding="utf-8")
    print(_dump({"receipt": str(receipt_path), "status": receipt["status"],
                 "totals": receipt["totals"], "exclusions": receipt["exclusions"]}), flush=True)
    return int(receipt["status"] != "COMPLETED")


if __name__ == "__main__":
    raise SystemExit(main())
