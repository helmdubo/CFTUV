#!/usr/bin/env python3
"""Analysis only: the fixed-bits enclosure midpoint is Q-linear.

The two converter functions are selected by AST, unmodified, from the audited
repository because importing lift.py pulls unavailable mpmath in this runtime.
This does not exercise materialization, Blender, or a native extension.
"""
from __future__ import annotations

import argparse
import ast
from fractions import Fraction
import hashlib
import json
from math import isqrt, lcm
from pathlib import Path
from random import Random
import sys


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--repo", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--seed", type=int, default=20261019)
    parser.add_argument("--samples", type=int, default=10000)
    args = parser.parse_args()
    repo = Path(args.repo).resolve()
    sys.path.insert(0, str(repo / "kernel/src"))
    from cftuv_envelope.exact_sqrt_sum import SqrtSumV1

    relpaths = (
        "kernel/src/cftuv_envelope/materialize/lift.py",
        "kernel/src/cftuv_envelope/exact_sqrt_sum.py",
        "native/cftuv-core/src/pyfloat.rs",
    )
    hashes = {rel: hashlib.sha256((repo / rel).read_bytes()).hexdigest() for rel in relpaths}
    module = ast.parse((repo / relpaths[0]).read_text())
    selected = ("enclosure_midpoint", "sqrt_sum_binary64")
    nodes = [node for node in module.body if isinstance(node, ast.FunctionDef) and node.name in selected]
    assert tuple(node.name for node in nodes) == selected
    namespace = {"SqrtSumV1": SqrtSumV1, "Fraction": Fraction, "ENCLOSURE_BITS": 64, "isqrt": isqrt, "lcm": lcm}
    exec(compile(ast.Module(body=nodes, type_ignores=[]), "lift.py:unmodified_AST_selected_functions", "exec"), namespace)
    midpoint = namespace["enclosure_midpoint"]
    convert = namespace["sqrt_sum_binary64"]
    rng = Random(args.seed)
    radicands = (1, 2, 3, 5, 6, 7, 10, 11, 13, 15)

    def value():
        return SqrtSumV1._from_map({q: Fraction(rng.randint(-1000000, 1000000), rng.randint(1, 10000)) for q in radicands})

    def project(v):
        return sum((coefficient * (Fraction(1) if q == 1 else Fraction(2 * isqrt(q << 128) + 1, 1 << 65)) for q, coefficient in v.terms), Fraction(0))

    failures = []
    tests = 0

    def check(label, x, slope, width0, width):
        nonlocal tests
        delta = width - width0
        current = x + slope.scaled(delta)
        projected = project(x) + project(slope) * delta
        numerator, denominator = midpoint(current)
        comparisons = {
            "rational_midpoint": projected == Fraction(numerator, denominator),
            "position_binary64": float(projected).hex() == convert(current).hex(),
            "uv_binary64": float(projected / width).hex() == convert(current, factor=1 / width).hex(),
        }
        # Packed rational hot loop: no radical, gcd, or reduction is needed per tick.
        intercept = project(x) - project(slope) * width0
        gradient = project(slope)
        common = lcm(intercept.denominator, gradient.denominator)
        a = intercept.numerator * (common // intercept.denominator)
        b = gradient.numerator * (common // gradient.denominator)
        top = a * width.denominator + b * width.numerator
        pos_den = common * width.denominator
        uv_den = common * width.numerator
        comparisons["packed_position_binary64"] = (top / pos_den).hex() == convert(current).hex()
        comparisons["packed_uv_binary64"] = (top / uv_den).hex() == convert(current, factor=1 / width).hex()
        tests += len(comparisons)
        for kind, ok in comparisons.items():
            if not ok:
                failures.append({"label": label, "kind": kind, "width": str(width), "width0": str(width0)})

    for sample in range(args.samples):
        x, slope = value(), value()
        width0 = Fraction(rng.randint(1, 1000000), rng.randint(1, 1000000))
        width = Fraction(rng.randint(1, 1000000), rng.randint(1, 1000000))
        check(sample, x, slope, width0, width)

    zero = SqrtSumV1.zero()
    check("zero", zero, zero, Fraction(1), Fraction(2))
    x = value()
    check("all_terms_cancel", x, -x, Fraction(1), Fraction(2))
    check("tie_even_down", SqrtSumV1.rational(Fraction((1 << 53) + 1, 1 << 53)), zero, Fraction(1), Fraction(1))
    check("tie_even_up", SqrtSumV1.rational(Fraction((1 << 53) + 3, 1 << 53)), zero, Fraction(1), Fraction(1))
    check("negative_underflow", SqrtSumV1.rational(Fraction(-1, 1 << 1200)), zero, Fraction(1), Fraction(1))

    result = {
        "audited_base": "f8f52ed75dce0ecb650d8f1f45dcb586597b1de0",
        "seed": args.seed,
        "random_samples": args.samples,
        "edge_samples": 5,
        "comparisons": tests,
        "failures": failures,
        "source_sha256": hashes,
        "selected_functions": selected,
        "function_loading": "AST-selected original function bodies, unmodified; full import unavailable due to absent mpmath",
        "python_version": sys.version,
        "q_basis": radicands,
        "bits": 64,
        "formula": "M64(sum c_q sqrt(q))=c_1+sum(q!=1)c_q*(2*isqrt(q<<128)+1)/2**65",
        "scope": "synthetic canonical affine scalar values, fixed conversion law; no full materialization, frozen scenes, budgets, source override, normals, native execution, or Blender",
        "limitation": "M64 is only Q-linear, not multiplicative, not order-preserving for arbitrarily tiny signs, and never an exact-predicate substitute",
    }
    Path(args.output).write_text(json.dumps(result, indent=2, ensure_ascii=False) + "\n")
    print(json.dumps({key: result[key] for key in ("random_samples", "edge_samples", "comparisons", "failures")}))
    return bool(failures)


if __name__ == "__main__":
    raise SystemExit(main())
