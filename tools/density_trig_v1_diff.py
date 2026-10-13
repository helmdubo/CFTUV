"""Finite owned trig AST/value/sign pair; old expanded controller parity вне scope."""

from __future__ import annotations

import argparse
from collections import Counter
from fractions import Fraction
import hashlib
import json
import lzma
import os
from pathlib import Path
import subprocess
import time

import sympy as sp

from cftuv_envelope.reference.density_construct_v1 import DensityConstructArenaV1, OwnedVec2V1, ExpandLimitsV1
from cftuv_envelope.reference.density_scalar_v2 import ScalarV2
from cftuv_envelope.reference.radical_v2 import value_of_wire
from cftuv_envelope.reference.density_trig_v1 import (
    TrigV1Error, owned_exact_q_trig, owned_principal_ray, owned_rational_pi_trig, principal_pi_multiple,
)
from density_construct_v1_diff import _legacy_staged_probe, _numeric_recipe
from density_scalar_v2_diff import _NumericReader, _vector_ids
from cftuv_envelope.reference.density_sign import _density_exact_sign
from cftuv_envelope.reference.common import ReferenceGeometryError


def queries():
    multiples = {Fraction(n, d) for d in (2, 3, 4, 5, 6, 8, 10, 12, 7, 15, 16, 18, 20, 24, 30, 36)
                 for n in range(-2 * d, 2 * d + 1)}
    for value in sorted(multiples):
        yield ("pi", f"{value.numerator}/{value.denominator}")
    for q in range(2, 7):
        yield "q", str(q)
    for ratio in (Fraction(0), Fraction(1, 4), Fraction(1, 2), Fraction(3, 4), Fraction(1)):
        for sign in ([0] if not ratio else [-1, 1]):
            for n in range(2, 7):
                for j in range(n + 1):
                    yield "ray", f"{ratio.numerator}/{ratio.denominator}", str(sign), str(n), str(j)
    yield "ray", "2/3", "1", "6", "0"
    yield "ray", "-1/1", "0", "7", "8"
    yield "ray", "-1/1", "0", "6", "7"
    yield "ray", "0/1", "1", "6", "1"
    yield "q", "7"


def query_text(case):
    return "\t".join(case)


def sign_record(arena, value):
    observation = _legacy_staged_probe(arena.to_sympy_readonly(value))
    if observation["sign"] is None:
        expanded = arena.finite_expand(value, ExpandLimitsV1(4096, 4, 65536))
        observation = _legacy_staged_probe(arena.to_sympy_readonly(expanded))
    return "UNDECIDABLE" if observation["sign"] is None else observation["sign"]


def old_producer_norm_sign(multiple):
    theta = sp.Rational(multiple.numerator, multiple.denominator) * sp.pi
    try:
        return _density_exact_sign(sp.cos(theta)**2 + sp.sin(theta)**2 - 1)
    except ReferenceGeometryError as error:
        if error.outcome.value != "REFERENCE_CERTIFIED_PREDICATE_UNDECIDABLE":
            raise
        return "UNDECIDABLE"


def record(arena, value):
    literal, scalar = arena.export_raw(value), arena.scalar(value)
    return {"canon": literal.canon, "nodes": [list(row) for row in literal.nodes],
            "root": literal.root, "numeric_v2_wire": None if scalar is None else list(scalar.value.wire()),
            "sign": sign_record(arena, value)}


def python_result(case):
    arena = DensityConstructArenaV1()
    try:
        if case[0] in ("pi", "axis"):
            multiple = Fraction(case[1])
            pair = owned_rational_pi_trig(arena, multiple)
        elif case[0] == "q":
            multiple = Fraction(1, int(case[1]))
            pair = owned_exact_q_trig(arena, int(case[1]))
        else:
            ratio, sign, n, j = Fraction(case[1]), *map(int, case[2:])
            pair = owned_principal_ray(arena, ratio, sign, n, j)
            angle = principal_pi_multiple(ratio, sign)
            multiple = None if angle is None else angle * Fraction(j, n)
    except TrigV1Error as error:
        names = {"INVALID_SUBDIVISION": "InvalidSubdivision", "INVALID_ORDINAL": "InvalidOrdinal",
                 "INCONSISTENT_DOT_SIGN": "InconsistentDotSign", "UNSUPPORTED_DENSITY_Q": "UnsupportedDensityQ"}
        return {"error": names[error.code.value]}, None
    if pair is None:
        return {"not_known": True}, None
    a, b = pair
    residual = arena.sub(arena.add(arena.integer_power(a, 2), arena.integer_power(b, 2)), arena.rational(1))
    result = {"cosine": record(arena, a), "sine": record(arena, b), "unit_residual": record(arena, residual)}
    if case[0] == "axis":
        basis = [arena.numeric(ScalarV2(value_of_wire(tuple(map(int, wire.split(','))) if wire else ())))
                 for wire in case[2:]]
        result["axes"] = [record(arena, arena.add(arena.mul(a, basis[index]), arena.mul(b, basis[index + 2])))
                          for index in range(2)]
    math_checks = []
    for value, function in zip(pair, (sp.cos, sp.sin), strict=True):
        expected = function(sp.Rational(multiple.numerator, multiple.denominator) * sp.pi)
        math_checks.append(sp.simplify(arena.to_sympy_readonly(value) - expected) == 0)
    math_checks.append(result["unit_residual"]["sign"] == old_producer_norm_sign(multiple))
    return result, math_checks


def native_run(exe, program, out):
    process = subprocess.Popen([str(exe)], stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                               stderr=subprocess.PIPE, text=True,
                               creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0))
    out.joinpath("native.pid").write_text(str(process.pid), encoding="utf-8")
    output, errors = process.communicate(program)
    out.joinpath("native.stdout").write_text(output, encoding="utf-8")
    out.joinpath("native.stderr").write_text(errors, encoding="utf-8")
    if process.returncode:
        raise RuntimeError(f"native exit{process.returncode}: {errors}")
    return process.pid, output.splitlines()


def compare(cases, rows, out):
    failures, counts = [], Counter()
    with out.joinpath("observations.jsonl").open("w", encoding="utf-8") as stream:
        for index, (case, text) in enumerate(zip(cases, rows, strict=True)):
            expected, mathematical = python_result(case)
            try:
                actual = json.loads(text)
            except json.JSONDecodeError:
                actual = text
            passed = expected == actual and (mathematical is None or all(mathematical))
            row = {"index": index, "input": query_text(case), "python": expected,
                   "rust": actual, "mathematical_component_checks": mathematical, "passed": passed}
            stream.write(json.dumps(row, separators=(",", ":")) + "\n")
            if not passed:
                failures.append(row)
            counts[case[0]] += 1
            for name in ("cosine", "sine", "unit_residual"):
                if name in expected:
                    counts[f"{name}_numeric" if expected[name]["numeric_v2_wire"] is not None
                           else f"{name}_opaque"] += 1
                    counts[f"{name}_sign_{expected[name]['sign']}"] += 1
    return failures, dict(counts)


def actual_zero_queries(turns, ledger, out):
    if turns is None or ledger is None:
        return [], []
    observations = [json.loads(line) for line in ledger.read_text(encoding="utf-8").splitlines()]
    wanted = {row["sequence"] for row in observations
              if row["old_readonly_staged_probe"]["sign"] == 0 and row["file"] == turns.name}
    document = json.loads(lzma.decompress(turns.read_bytes()))
    reader = _NumericReader(document["expressions"])
    cases, receipts = [], []
    for row in document["records"]:
        if row["sequence"] not in wanted:
            continue
        arena = DensityConstructArenaV1()
        body, before, after = row["input"], row["input"]["metric"], row["outcome"]["memo_after"]
        vectors = [OwnedVec2V1(*(arena.numeric(reader.read(index)) for index in
                                _vector_ids(body[name], before, after))) for name in ("incoming", "outgoing")]
        grid = before["base"]["gram"]
        gram = tuple(arena.numeric(reader.read(index)) for index in (grid[0][0], grid[0][1], grid[1][1]))
        facts, (incoming, normal) = _numeric_recipe(arena, gram, *vectors, before["base"]["sign"], body["count"])
        angle = (Fraction(*body["canonical"][1:]) if body["canonical"] is not None else
                 principal_pi_multiple(arena.whole_rational(facts["cosine_squared"]),
                                       _legacy_staged_probe(arena.to_sympy_readonly(facts["raw_dot"]))["sign"]))
        if angle is None or body["rotation"] is not None:
            raise AssertionError("Old zero observation is outside special principal recipe")
        multiple = angle / (body["count"] + 1)
        basis = (incoming.x, incoming.y, arena.scaled(normal.x, body["orientation"]),
                 arena.scaled(normal.y, body["orientation"]))
        wires = [",".join(map(str, arena.scalar(value).value.wire())) for value in basis]
        case = ("axis", f"{multiple.numerator}/{multiple.denominator}", *wires)
        result, _ = python_result(case)
        axis = result["axes"][0]
        if axis["numeric_v2_wire"] != [] or axis["sign"] != 0:
            raise AssertionError("Actual previously opaque zero not closed by finite trig island")
        cases.append(case)
        receipts.append({"sequence": row["sequence"], "multiple": str(multiple),
                         "input": query_text(case), "axis_x": axis,
                         "prior_observation": "old rational zero; prior candidate needs owned expanded view"})
    if len(cases) != len(wanted):
        raise AssertionError("Missing actual zero observations")
    out.joinpath("actual-zero-observations.json").write_text(json.dumps(receipts, indent=2), encoding="utf-8")
    return cases, receipts


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--exe", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--actual-turns", type=Path)
    parser.add_argument("--prior-ledger", type=Path)
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    args.out.joinpath("python.pid").write_text(str(os.getpid()), encoding="utf-8")
    actual_cases, actual_receipts = actual_zero_queries(args.actual_turns, args.prior_ledger, args.out)
    cases = list(queries()) + actual_cases
    program = "\n".join(map(query_text, cases)) + "\n"
    args.out.joinpath("inputs.tsv").write_text(program, encoding="utf-8")
    started = time.perf_counter()
    pid, rows = native_run(args.exe, program, args.out)
    failures, counts = compare(cases, rows, args.out)
    receipt = {"scope": "finite owned trig Numeric islands and unchanged raw opaque admission",
               "cases": len(cases), "mismatches": len(failures), "failures": failures,
               "counts": counts, "native_pid": pid, "python_pid": os.getpid(),
               "seconds": time.perf_counter() - started,
               "binary_sha256": hashlib.sha256(args.exe.read_bytes()).hexdigest(),
               "input_sha256": hashlib.sha256(program.encode()).hexdigest(),
               "actual_prior_zero_observations_closed": len(actual_receipts),
               "sign_boundary": "unchanged iv160 then actual owned finite expansion then old quadratic reader; no SymPy expansion of new export",
               "excluded": ["old AST bytes", "whole old controller/field decisions", "nested-root Numeric admission", "product flip"]}
    args.out.joinpath("receipt.json").write_text(json.dumps(receipt, indent=2), encoding="utf-8")
    print(json.dumps({key: value for key, value in receipt.items() if key != "failures"}), flush=True)
    return int(bool(failures))


if __name__ == "__main__":
    raise SystemExit(main())
