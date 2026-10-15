"""Парный A2 integer value-path и независимая проверка ранга через count_roots.

Нет допуска старого evaluator, product switch или границы композиции. Внешний
Rust replay неизменён; все исходные строки и наблюдения сохраняются для повтора.
"""

from __future__ import annotations

import argparse
from fractions import Fraction
import hashlib
import json
import os
from pathlib import Path
import random
import subprocess
import sys
import time

from cftuv_envelope.reference.density_angle_v1 import (
    AngleAtomV1, AngleV1Error, AngleV1ErrorCode, COMPONENT_BITS, root_bits,
)
from density_angle_diff import polynomial, root_status


def rational_text(value):
    return f"{value.numerator}/{value.denominator}"


def atoms(samples):
    ratios = [Fraction(0), Fraction(1, 4), Fraction(81, 256), Fraction(1, 64),
              Fraction(9, 16), Fraction(1, 2**512), Fraction(2**512 - 1, 2**512)]
    rng = random.Random(0xA231012)
    for _ in range(samples):
        denominator = rng.randrange(2, 10**12)
        ratios.append(Fraction(rng.randrange(1, denominator), denominator))
    for n in range(1, 7):
        for ratio in ratios:
            for sign in ([0] if not ratio else [-1, 1]):
                yield n, ratio, sign


def queries(samples):
    for n, ratio, sign in atoms(samples):
        for bits in (0, 2, 32, 160):
            yield "bound", n, ratio, sign, str(bits)
        if n >= 2:
            for ordinal in range(1, n):
                yield "ray", n, ratio, sign, f"{ordinal},{root_bits(n)}"
                if ratio in (Fraction(0), Fraction(1, 4), Fraction(81, 256)):
                    for bits in (0, 1, 2, 8, 32):
                        yield "ray", n, ratio, sign, f"{ordinal},{bits}"
    yield from invalid_queries()


def invalid_queries():
    for n, ratio, sign in [(0, Fraction(0), 0), (7, Fraction(0), 0),
                           (2, Fraction(-1, 4), -1), (2, Fraction(1), 1),
                           (2, Fraction(0), 1), (2, Fraction(1, 4), 0)]:
        yield "bound", n, ratio, sign, "32"
    yield "ray", 4, Fraction(1, 4), -1, f"0,{2**32 - 1}"
    yield "ray", 4, Fraction(1, 4), -1, "4,32"
    yield "ray", 4, Fraction(1, 4), -1, f"1,{2**31}"
    yield "bound", 3, Fraction(81, 256), -1, str(2**32 - 1)


def query_text(case):
    operation, n, ratio, sign, argument = case
    return f"{operation}\t{n}\t{rational_text(ratio)}\t{sign}\t{argument}"


def error_text(case, error):
    operation, n, _, sign, argument = case
    names = {
        AngleV1ErrorCode.UNSUPPORTED_SUBDIVISION: f"UnsupportedSubdivision({n})",
        AngleV1ErrorCode.RATIO_OUTSIDE_HALF_OPEN_UNIT_INTERVAL: "RatioOutsideHalfOpenUnitInterval",
        AngleV1ErrorCode.INCONSISTENT_DOT_SIGN: f"InconsistentDotSign({sign})",
    }
    if error.code in names:
        return f"INPUT_ERROR\t{names[error.code]}"
    if error.code is AngleV1ErrorCode.INVALID_ORDINAL:
        ordinal = argument.split(",")[0]
        return f"RAY_ERROR\tInvalidOrdinal {{ ordinal: {ordinal}, subdivision: {n} }}"
    if error.code is AngleV1ErrorCode.BIT_COUNT_OVERFLOW:
        return ("RAY_ERROR\tInternalInterval(BitCountOverflow)" if operation == "ray"
                else "INTERNAL_ERROR\tBitCountOverflow")
    raise error


def python_result(case):
    operation, n, ratio, sign, argument = case
    try:
        atom = AngleAtomV1(n, ratio, sign)
        if operation == "bound":
            root = atom.enclose(int(argument))
            return {"kind": "bound", "root": root, "ray": None}
        ordinal, bits = map(int, argument.split(","))
        ray = atom.ray(ordinal) if bits == root_bits(n) else atom.ray_enclosure(ordinal, bits)
        return {"kind": "ray", "root": ray.root, "ray": ray}
    except AngleV1Error as error:
        return {"kind": "error", "text": error_text(case, error)}


def paired_checks(case, value, line):
    if value["kind"] == "error":
        return {"typed_error_and_order": line == value["text"]}
    fields, root = line.split("\t"), value["root"]
    if value["kind"] == "bound":
        if len(fields) != 12 or fields[0] != "BOUND":
            return {"bound_protocol": False}
        lower, upper = map(Fraction, fields[1:3])
        return {"root_endpoints": (lower, upper) == (root.lower, root.upper),
                "degree": int(fields[4]) == root.stats.polynomial_degree,
                "bisections": int(fields[6]) == root.stats.bisections}
    ray = value["ray"]
    endpoints = [ray.cosine.lower, ray.cosine.upper, ray.sine.lower, ray.sine.upper,
                 root.lower, root.upper]
    expected = "\t".join(["RAY", *map(rational_text, endpoints), str(root.stats.bisections)])
    bits = int(case[4].split(",")[1])
    checks = {"full_ray_literal": line == expected, "positive_sine": ray.sine.lower >= 0}
    if bits == root_bits(case[1]):
        limit = Fraction(1, 2**COMPONENT_BITS)
        checks["fixed_component_widths"] = ray.cosine.width < limit and ray.sine.width < limit
    return checks


def independent_rank(case, value, poly_cache):
    if value["kind"] == "error":
        return None
    operation, n, ratio, sign, argument = case
    wanted = (operation == "bound" and argument == "32") or (
        operation == "ray" and argument == f"1,{root_bits(n)}")
    if not wanted:
        return None
    poly = poly_cache.setdefault((n, ratio), polynomial(n, ratio))
    root = value["root"]
    gl, rl = root_status(poly, root.lower)
    gu, _ = root_status(poly, root.upper)
    rank = 2 if sign < 0 else 1
    target = bool(rl and gl == rank - 1) if root.lower == root.upper else gl >= rank > gu
    return {"principal_rank": target, "closed_root_bound": -1 <= root.lower <= root.upper <= 1}


def principal_floor_counterexample():
    ratio = Fraction(1, 4)
    atom, coordinate = AngleAtomV1(4, ratio, -1), Fraction(1, 2)
    poly = polynomial(4, ratio)
    above, at_root = root_status(poly, coordinate)
    return {"n": 4, "ratio": "1/4", "sign": -1, "coordinate": "1/2",
            "polynomial_root": bool(at_root), "roots_above": above,
            "comparison": atom.compare(coordinate), "target_rank": 2,
            "passed": bool(at_root and above != 1 and atom.compare(coordinate) == -1)}


def run_native(exe, program, out):
    process = subprocess.Popen([str(exe)], stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                               stderr=subprocess.PIPE, text=True,
                               creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0))
    out.joinpath("native.pid").write_text(str(process.pid), encoding="utf-8")
    output, errors = process.communicate(program)
    out.joinpath("native.stdout").write_text(output, encoding="utf-8")
    out.joinpath("native.stderr").write_text(errors, encoding="utf-8")
    return process.pid, process.returncode, output.splitlines(), errors


def evaluate(cases, rows, out):
    failures, poly_cache, rank_count, kinds = [], {}, 0, {}
    with out.joinpath("observations.jsonl").open("w", encoding="utf-8") as stream:
        for index, (case, row) in enumerate(zip(cases, rows, strict=True)):
            value = python_result(case)
            kinds[value["kind"]] = kinds.get(value["kind"], 0) + 1
            checks = paired_checks(case, value, row)
            independent = independent_rank(case, value, poly_cache)
            if independent is not None:
                rank_count += 1
                checks.update(independent)
            observation = {"index": index, "input": query_text(case),
                           "native": row, "checks": checks}
            stream.write(json.dumps(observation) + "\n")
            if not all(checks.values()):
                failures.append(observation)
    return failures, rank_count, kinds


def source_hashes():
    root = Path(__file__).resolve().parents[1]
    paths = ["kernel/src/cftuv_envelope/reference/density_angle_v1.py",
             "kernel/tests/test_density_angle_v1.py", "tools/density_angle_v1_diff.py",
             "native/cftuv-density/src/angle_monotone.rs",
             "native/cftuv-density/src/angle_profile.rs", "native/cftuv-density/src/angle.rs",
             "native/cftuv-density/src/interval.rs", "native/cftuv-density/src/angle_poly.rs"]
    return {path: hashlib.sha256(root.joinpath(path).read_bytes()).hexdigest() for path in paths}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--exe", type=Path, required=True)
    parser.add_argument("--samples", type=int, default=6)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    sys.set_int_max_str_digits(0)
    args.out.mkdir(parents=True, exist_ok=True)
    args.out.joinpath("python.pid").write_text(str(os.getpid()), encoding="utf-8")
    cases = list(queries(args.samples))
    program = "\n".join(map(query_text, cases)) + "\n"
    args.out.joinpath("inputs.tsv").write_text(program, encoding="utf-8")
    start = time.perf_counter()
    native_pid, status, rows, errors = run_native(args.exe, program, args.out)
    native_seconds = time.perf_counter() - start
    if status or len(rows) != len(cases):
        raise RuntimeError(f"Native status={status}, rows={len(rows)}/{len(cases)}, stderr={errors}")
    failures, rank_count, kinds = evaluate(cases, rows, args.out)
    branch = principal_floor_counterexample()
    if not branch["passed"]:
        failures.append({"principal_floor_counterexample": branch})
    receipt = {"scope": "A2 fixed component integer value-path only",
               "excluded": ["legacy admission/sign controller", "DensityWork budgets",
                            "arbitrary composition bound", "product default switch"],
               "cases": len(cases), "kinds": kinds, "mismatches": len(failures),
               "independent_rank_checks": rank_count, "failures": failures,
               "fixed_bits": {n: root_bits(n) for n in range(2, 7)},
               "component_bits": COMPONENT_BITS, "branch_counterexample": branch,
               "python_pid": os.getpid(), "native_pid": native_pid,
               "native_seconds": native_seconds, "seconds": time.perf_counter() - start,
               "binary_sha256": hashlib.sha256(args.exe.read_bytes()).hexdigest(),
               "input_sha256": hashlib.sha256(program.encode()).hexdigest(),
               "source_sha256": source_hashes()}
    args.out.joinpath("receipt.json").write_text(json.dumps(receipt, indent=2), encoding="utf-8")
    print(json.dumps({key: value for key, value in receipt.items()
                      if key not in ("failures", "source_sha256")}), flush=True)
    return int(bool(failures))


if __name__ == "__main__":
    raise SystemExit(main())
