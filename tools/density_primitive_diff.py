"""Независимый дифференциал точного R3b-листа против функций Python-эталона.

    python tools/density_primitive_diff.py --exe PATH/TO/replay.exe --samples 200 --out receipt.json

Это проверка листа, не всей стадии PLAN_COMPILE. V2 Work выводится Rust, но не сравнивается:
наблюдаемая цена R3 принадлежит поиску DensityExactWorkBudget, которого в этих операциях нет.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import json
from pathlib import Path
import random
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "kernel" / "src"))
sys.set_int_max_str_digits(0)

import sympy as sp

from cftuv_envelope.reference import adaptive_density_fan as fan
from cftuv_envelope.reference import angular
from cftuv_envelope.reference.metric import _DensityExactMemo
from cftuv_envelope.reference.planar_types import exact_quadratic_value
from cftuv_envelope.reference.radical_v2 import RadicalSumV2, canonical_text, terms_wire


@dataclass
class OracleMetric:
    gram: tuple
    inverse_gram: tuple
    owner_orientation_sign: int
    _density_exact_memo: _DensityExactMemo

    @classmethod
    def make(cls, entries, owner):
        a, b, c = map(sp.Rational, entries)
        determinant = a * c - b * b
        return cls(((a, b), (b, c)), ((c / determinant, -b / determinant), (-b / determinant, a / determinant)), owner, _DensityExactMemo())

    def density_expressions(self, vector):
        return vector.expressions()


def v2_value(expression):
    value = exact_quadratic_value(sp.expand(expression))
    return RadicalSumV2.from_terms(value.terms)[0]


def value_wire(expression):
    return list(terms_wire(v2_value(expression)))


def oracle(op, metric, left, right, arg):
    if op == "gram_dot":
        return ["VALUE", value_wire(angular._density_dot_expression(metric, left, right))]
    if op == "dual_dot":
        return ["VALUE", value_wire(fan._dual_dot(metric, left, right))]
    if op == "cross":
        return ["VALUE", value_wire(fan._oriented_cross(metric, left, right))]
    if op == "quarter_turn":
        x, y = fan._quarter_turn(metric, left, arg).expressions()
        return ["VECTOR", value_wire(x), value_wire(y)]
    if op == "sign":
        return ["SIGN", angular._density_exact_sign(left.x, metric)]
    function = fan._subturn if op == "subturn" else fan._subturn_boundary
    return ["BOOL", int(function(metric, left, right, arg))]


def parse_output(line):
    fields = line.rstrip("\r\n").split("\t")
    wire = lambda value: [] if value == "-" else [int(item) for item in value.split(",")]
    if fields[0] == "VALUE":
        return [fields[0], wire(fields[1])]
    if fields[0] == "VECTOR":
        return [fields[0], wire(fields[1]), wire(fields[2])]
    if fields[0] in ("BOOL", "SIGN"):
        return [fields[0], int(fields[1])]
    return fields


def inputs(samples, seed):
    rng = random.Random(seed)
    # Точные границы q, нули, отрицательный dot, знаки ориентации и косой Gram.
    vectors = ((0, 0), (1, 0), (0, 1), (-1, 0), (1, 1), (sp.sqrt(3), 1), (1, sp.sqrt(3)))
    for owner in (-1, 1):
        for left in vectors:
            for right in vectors:
                yield (1, 0, 1), owner, left, right
    for _ in range(samples):
        a, b, c = rng.randint(1, 50), rng.randint(-20, 20), rng.randint(1, 50)
        gram = (sp.Rational(a * a + b * b, 7), sp.Rational(b * c, 7), sp.Rational(c * c, 7))
        scalar = lambda: sp.Rational(rng.randint(-8, 8), rng.randint(1, 13)) + rng.choice((0, sp.sqrt(2), -sp.sqrt(3), sp.sqrt(5)))
        yield gram, rng.choice((-1, 1)), (scalar(), scalar()), (scalar(), scalar())


def program(samples, seed):
    lines, expected = [], []
    for gram, owner, left, right in inputs(samples, seed):
        metric = OracleMetric.make(gram, owner)
        lv, rv = fan._vector(*left), fan._vector(*right)
        coordinates = [canonical_text(v2_value(value)) for value in (*left, *right)]
        operations = [(name, "-") for name in ("gram_dot", "dual_dot", "cross", "sign")]
        operations += [("quarter_turn", orientation) for orientation in (-1, 1)]
        operations += [(name, q) for q in range(2, 7) for name in ("subturn", "subturn_boundary")]
        # Повтор той же транзакции ловит асимметричную/ошибочную память результата.
        for op, arg in operations + operations:
            values = [op, *(f"{sp.Rational(value).p}/{sp.Rational(value).q}" for value in gram), owner, *coordinates, arg]
            lines.append("\t".join(map(str, values)))
            expected.append(oracle(op, metric, lv, rv, arg))
        lines.append("reset")
        expected.append(["RESET"])
    return lines, expected


def compare(executable, lines, expected, pid_file):
    process = subprocess.Popen([str(executable)], stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, encoding="utf-8")
    with Path(pid_file).open("a", encoding="utf-8") as log:
        log.write(f"{process.pid}\tdensity-primitive-replay\n")
    stdout, stderr = process.communicate("\n".join(lines) + "\n")
    actual = [parse_output(line) for line in stdout.splitlines()]
    mismatches = []
    for index in range(max(len(actual), len(expected))):
        wanted = expected[index] if index < len(expected) else None
        found = actual[index] if index < len(actual) else None
        if wanted != found:
            mismatches.append({"row": index, "input": lines[index] if index < len(lines) else None, "expected": wanted, "actual": found})
    return {"schema": "cftuv.density-primitive-parity.v1", "rows": len(expected),
            "operations": sum(row != ["RESET"] for row in expected), "transaction_resets": expected.count(["RESET"]),
            "mismatch_count": len(mismatches), "mismatches": mismatches[:20], "exit_code": process.returncode,
            "stderr": stderr, "v2_work_compared": False}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--exe", type=Path, required=True)
    parser.add_argument("--samples", type=int, default=200)
    parser.add_argument("--seed", type=int, default=20261012)
    parser.add_argument("--out", type=Path, required=True)
    options = parser.parse_args()
    lines, expected = program(options.samples, options.seed)
    receipt = compare(options.exe, lines, expected, options.out.with_suffix(".pids.log"))
    options.out.write_text(json.dumps(receipt, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({key: value for key, value in receipt.items() if key not in ("mismatches", "stderr")}))
    return int(receipt["mismatch_count"] != 0 or receipt["exit_code"] != 0)


if __name__ == "__main__":
    raise SystemExit(main())
