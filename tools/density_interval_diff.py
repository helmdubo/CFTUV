"""Независимые Fraction/root-count проверки A1/A2 primitives, без product flip."""

from __future__ import annotations

import argparse
from fractions import Fraction as F
import hashlib
import json
from math import isqrt
from pathlib import Path
import random
import subprocess
import sys

import sympy as sp

sys.set_int_max_str_digits(0)
SCALE = 10**27


def text(value):
    value = F(value)
    return f"{value.numerator}/{value.denominator}"


def units(lower, upper):
    return (lower.numerator * SCALE // lower.denominator,
            -((-upper.numerator * SCALE) // upper.denominator))


def operation_cases(samples):
    rng = random.Random(0xA12012)
    intervals = [(F(-1), F(1)), (F(0), F(0)), (F(1657, 5120), F(1657, 5120)),
                 (F(-3647, 5120), F(-3647, 5120)), (F(1, 2**513), F(1, 2**511))]
    intervals += [tuple(sorted(F(rng.randrange(-10**30, 10**30), rng.randrange(1, 10**30))
                              for _ in range(2))) for _ in range(samples)]
    for index, (a, b) in enumerate(intervals):
        c, d = intervals[(index * 7 + 3) % len(intervals)]
        product = [a*c, a*d, b*c, b*d]
        cases = [("add", (a+c, b+d)), ("sub", (a-d, b-c)),
                 ("mul", (min(product), max(product))),
                 ("div", None if c <= 0 <= d else
                  (min(a/c, a/d, b/c, b/d), max(a/c, a/d, b/c, b/d)))]
        for name, expected in cases:
            yield "\t".join(["interval_"+name, text(a), text(b), text(c), text(d)]), expected
        square = (min(a*a, b*b) if not a <= 0 <= b else F(0), max(a*a, b*b))
        yield f"interval_square\t{text(a)}\t{text(b)}\t-\t-", square
        yield f"interval_scale\t{text(a)}\t{text(b)}\t-\t-", (a, b)
        if a >= 0:
            for bits in (0, 27, 160, 300):
                factor = 2**bits
                lower = isqrt(a.numerator * factor**2 // a.denominator)
                upper = isqrt(b.numerator * factor**2 // b.denominator)
                if upper**2 * b.denominator < b.numerator * factor**2:
                    upper += 1
                yield f"interval_sqrt\t{text(a)}\t{text(b)}\t{bits}\t-", (F(lower, factor), F(upper, factor))


def exact_cosine_is_inside(lower, upper, angle):
    # T_(2b)(x)-1 имеет square-free корни cos(k*pi/b), k=0..b.
    # Целевой корень cos(a*pi/b) имеет ровно a корней строго выше.
    x = sp.Symbol("x")
    poly = sp.Poly(sp.chebyshevt(2*angle.denominator, x)-1, x).sqf_part()
    def above(value):
        q = sp.Rational(value.numerator, value.denominator)
        return int(poly.count_roots(q, 1)) - int(poly.eval(q) == 0)
    def root(value):
        return poly.eval(sp.Rational(value.numerator, value.denominator)) == 0
    rank = angle.numerator
    return (above(lower) > rank or (root(lower) and above(lower) == rank)) and above(upper) <= rank


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--exe", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--samples", type=int, default=120)
    args = parser.parse_args()
    arithmetic = list(operation_cases(args.samples))
    rays = [(f"ray\t{n}\t{text(r)}\t{sign}\t{j},{bits}", theta*j/n)
            for n in range(2, 7) for j in range(1, n) for bits in (8, 160)
            for r, sign, theta in [(F(0), 0, F(1, 2)), (F(1, 4), 1, F(1, 3)), (F(1, 4), -1, F(2, 3))]]
    program = "\n".join(row for row, _ in arithmetic+rays)+"\n"
    process = subprocess.Popen([str(args.exe)], stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                               stderr=subprocess.PIPE, text=True)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.with_suffix(".pid").write_text(str(process.pid), encoding="utf-8")
    output, errors = process.communicate(program)
    rows = output.splitlines()
    failures = []
    if process.returncode or len(rows) != len(arithmetic)+len(rays):
        failures.append({"exit": process.returncode, "rows": len(rows), "stderr": errors})
    for index, ((request, expected), row) in enumerate(zip(arithmetic, rows)):
        fields = row.split("\t")
        if expected is None:
            valid = "DenominatorContainsZero" in row
        else:
            valid = (len(fields) == 5 and fields[0] == "INTERVAL" and
                     tuple(map(F, fields[1:3])) == expected and
                     tuple(map(int, fields[3:5])) == units(*expected))
        if not valid:
            failures.append({"index": index, "input": request, "actual": row})
    for (request, angle), row in zip(rays, rows[len(arithmetic):]):
        fields = row.split("\t")
        valid = len(fields) == 8 and fields[0] == "RAY"
        if valid:
            cl, ch, sl, sh = map(F, fields[1:5])
            # sin(a*pi)=cos((1/2-a)*pi); cosine чётен.
            valid = (-1 <= cl <= ch <= 1 and 0 <= sl <= sh <= 1 and
                     exact_cosine_is_inside(cl, ch, angle) and
                     exact_cosine_is_inside(sl, sh, abs(F(1, 2)-angle)))
        if not valid:
            failures.append({"input": request, "actual": row})
    receipt = {"scope": "A1/A2 prototypes only; no legacy windows/admission parity",
               "arithmetic_cases": len(arithmetic), "ray_cases": len(rays),
               "mismatches": len(failures), "failures": failures,
               "binary_sha256": hashlib.sha256(args.exe.read_bytes()).hexdigest(),
               "input_sha256": hashlib.sha256(program.encode()).hexdigest(), "pid": process.pid}
    args.out.write_text(json.dumps(receipt, indent=2), encoding="utf-8")
    print(json.dumps({key: value for key, value in receipt.items() if key != "failures"}), flush=True)
    return int(bool(failures))


if __name__ == "__main__":
    raise SystemExit(main())
