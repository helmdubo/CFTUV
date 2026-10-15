"""Независимая exact-проверка ранга AngleAtom через SymPy count_roots.

Это проверка числового прототипа A2, не паритет допуска старого iv160 и не
разрешение менять authority windows. Нет численного cos/acos и float-эталона.
"""

from __future__ import annotations

import argparse
from fractions import Fraction
import hashlib
import json
from pathlib import Path
import random
import subprocess
import sys
import time

import sympy as sp

sys.set_int_max_str_digits(0)


def rational_text(value):
    value = Fraction(value)
    return f"{value.numerator}/{value.denominator}"


def cases(samples):
    rng = random.Random(0xA23012)
    ratios = [Fraction(0), Fraction(1, 4), Fraction(81, 256),
              Fraction(1, 2**512), Fraction(2**512 - 1, 2**512)]
    for _ in range(samples):
        denominator = rng.randrange(2, 10**12)
        ratios.append(Fraction(rng.randrange(1, denominator), denominator))
    for n in range(1, 7):
        for ratio in ratios:
            for direction in ([0] if ratio == 0 else [-1, 1]):
                for bits in ([0, 8, 32, 160] if ratio in ratios[:5] else [64]):
                    yield n, ratio, direction, bits


def polynomial(n, ratio):
    x = sp.Symbol("x")
    t = sp.chebyshevt(n, x)
    expression = t if not ratio else ratio.denominator * t**2 - ratio.numerator
    return sp.Poly(expression, x, domain=sp.QQ)


def root_status(poly, value):
    x = sp.Rational(value.numerator, value.denominator)
    at_root = int(poly.eval(x) == 0)
    # count_roots считает оба конца; x вычитается, 1 не является корнем.
    return int(poly.count_roots(x, 1)) - at_root, at_root


def verify(case, line):
    n, ratio, direction, bits = case
    fields = line.split("\t")
    if len(fields) != 12 or fields[0] != "BOUND":
        return {"reason": "not a complete bound", "line": line}
    lower, upper = map(Fraction, fields[1:3])
    rank, degree, sturm_len, steps, variations, gl, gu, rl, ru = map(int, fields[3:])
    poly = polynomial(n, ratio)
    wanted_rank = 2 if direction < 0 else 1
    oracle_gl, oracle_rl = root_status(poly, lower)
    oracle_gu, oracle_ru = root_status(poly, upper)
    checks = {
        "rank": rank == wanted_rank,
        "degree": degree == poly.degree(),
        "bounds": -1 <= lower <= upper <= 1,
        "width": upper - lower <= Fraction(1, 2**bits),
        "counts": (gl, gu, rl, ru) == (oracle_gl, oracle_gu, oracle_rl, oracle_ru),
        "steps": 1 <= steps <= bits + 1 and variations == steps,
        "sturm_length": 2 <= sturm_len <= degree + 1,
    }
    if lower == upper:
        checks["target"] = bool(oracle_rl and oracle_gl == wanted_rank - 1)
    else:
        checks["target"] = oracle_gl >= wanted_rank and oracle_gu < wanted_rank
        checks["all_bisections"] = steps == bits + 1
    return None if all(checks.values()) else {"checks": checks, "line": line}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--exe", type=Path, required=True)
    parser.add_argument("--samples", type=int, default=12)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    inputs = list(cases(args.samples))
    program = "\n".join(f"bound\t{n}\t{rational_text(ratio)}\t{direction}\t{bits}"
                        for n, ratio, direction, bits in inputs) + "\n"
    started = time.perf_counter()
    process = subprocess.Popen([str(args.exe)], stdin=subprocess.PIPE,
                               stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.with_suffix(".pid").write_text(str(process.pid), encoding="utf-8")
    output, errors = process.communicate(program)
    rust_seconds = time.perf_counter() - started
    rows = output.splitlines()
    failures = []
    if process.returncode or len(rows) != len(inputs):
        failures.append({"exit": process.returncode, "rows": len(rows),
                         "expected_rows": len(inputs), "stderr": errors})
    for index, (case, row) in enumerate(zip(inputs, rows)):
        failure = verify(case, row)
        if failure:
            failures.append({"index": index, "input": list(map(str, case)), **failure})
    receipt = {"scope": "A2 exact rank enclosure only; not legacy admission parity",
               "cases": len(inputs), "mismatches": len(failures), "failures": failures,
               "rust_seconds": rust_seconds, "total_seconds": time.perf_counter() - started,
               "binary_sha256": hashlib.sha256(args.exe.read_bytes()).hexdigest(),
               "input_sha256": hashlib.sha256(program.encode()).hexdigest(),
               "sympy_version": sp.__version__, "pid": process.pid}
    args.out.write_text(json.dumps(receipt, indent=2), encoding="utf-8")
    print(json.dumps({key: value for key, value in receipt.items() if key != "failures"}), flush=True)
    return int(bool(failures))


if __name__ == "__main__":
    raise SystemExit(main())
