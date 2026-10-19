"""Small proof-review probes for CFTUV f8f52ed7. No Blender or product edits.

Two dependency-heavy pure functions are executed verbatim from their AST nodes;
the exact arithmetic and work-budget classes are imported from the pinned source.
These are synthetic counterexamples, not reproduction of the field domain.
"""
from __future__ import annotations

import ast
import argparse
from fractions import Fraction
import hashlib
import json
import math
from pathlib import Path
import sys

HERE = Path(__file__).resolve().parent
PARSER = argparse.ArgumentParser(description=__doc__)
PARSER.add_argument("--root", type=Path, required=True, help="Pinned CFTUV repository root")
PARSER.add_argument("--output", type=Path, required=True, help="JSON report path (use scratch for reruns)")
ARGS = PARSER.parse_args()
REPO = ARGS.root.resolve()
SRC = REPO / "kernel/src"
sys.path.insert(0, str(SRC))

from cftuv_envelope._cpython311 import left_fold_sum
from cftuv_envelope.exact_sqrt_sum import SqrtSumV1, exact_work_budget


def pure_function(relative, name, environment):
    path = SRC / relative
    raw = path.read_bytes()
    tree = ast.parse(raw, filename=str(path))
    node = next(item for item in tree.body if isinstance(item, ast.FunctionDef) and item.name == name)
    module = ast.Module(body=[node], type_ignores=[])
    exec(compile(module, str(path), "exec"), environment)
    return environment[name], {
        "path": str(path.relative_to(REPO)),
        "function": name,
        "line": node.lineno,
        "end_line": node.end_lineno,
        "source_sha256": hashlib.sha256(raw).hexdigest(),
        "execution": "verbatim AST function; dependencies supplied explicitly",
    }


def frac(value):
    return str(Fraction(value))


def main():
    reports = {}
    env = {"math": math, "Fraction": Fraction, "left_fold_sum": left_fold_sum}
    fit, source = pure_function("cftuv_envelope/materialize/silhouette.py", "uv_fit_residual", env)
    within, _ = pure_function("cftuv_envelope/materialize/silhouette.py", "_within", env)
    chart = [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0)]
    k, tau = Fraction(1, 1024), Fraction(1, 256)
    uv_rows = []
    for width in (Fraction(1, 32), Fraction(1, 16), Fraction(1, 8)):
        uvs = [(0.0, 0.0), (0.0, 0.0), (float(k / width), 0.0), (0.0, 0.0)]
        residual = fit(chart, uvs)
        expected = k / (4 * width)
        assert Fraction(residual) == expected
        assert within(residual, tau) == (width >= Fraction(1, 16))
        uv_rows.append({"width": frac(width), "residual": frac(residual), "accepted": within(residual, tau)})
    reports["synthetic_uv_fit_threshold"] = {
        "source": source,
        "raw_U": ["0", "0", frac(k), "0"],
        "raw_V": ["0"] * 4,
        "law": "U=raw_U/width; least squares residual=k/(4*width)",
        "ideal_exact_event": "1/16",
        "tau": frac(tau),
        "rows": uv_rows,
    }

    class UnexpectedNonAffine(Exception):
        pass

    second_difference, source = pure_function(
        "cftuv_envelope/materialize/parametric.py",
        "_second_difference_slope",
        {"SqrtSumV1": SqrtSumV1, "Fraction": Fraction, "ParametricRefusal": UnexpectedNonAffine},
    )
    def f(width):
        return width + (width - 1) * (width - 2) * (width - 3)
    low, mid, high = [SqrtSumV1.rational(f(Fraction(w))) for w in (1, 2, 3)]
    slope = second_difference(low, mid, high, Fraction(1), "NONAFFINE")
    assert slope == SqrtSumV1.rational(1)
    other = Fraction(5, 2)
    assert f(other) != other
    reports["three_point_counterexample"] = {
        "source": source,
        "function": "w+(w-1)*(w-2)*(w-3)",
        "samples": ["1", "2", "3"],
        "helper_accepts_slope": "1",
        "other_width": frac(other),
        "true_value": frac(f(other)),
        "predicted_value": frac(other),
        "scope": "Refutes an unrestricted inference; does not refute a separately proved degree<=2 numerator / degree<=1 denominator law.",
    }

    p = lambda w: (w - Fraction(1, 3)) ** 2
    q = lambda w: (w - Fraction(1, 4)) * (w - Fraction(3, 4))
    reports["root_isolation_counterexamples"] = {
        "even_root": {"polynomial": "(w-1/3)^2", "endpoint_values": [frac(p(Fraction(0))), frac(p(Fraction(1)))], "missed_event": "1/3"},
        "two_roots": {"polynomial": "(w-1/4)*(w-3/4)", "endpoint_values": [frac(q(Fraction(0))), frac(q(Fraction(1)))], "missed_events": ["1/4", "3/4"]},
        "pole": {"function": "1/(w-1/2)", "endpoint_values": ["-2", "2"], "no_zero": True, "pole_event": "1/2"},
        "requirement": "Count and isolate all roots, including even multiplicities, and separately certify nonzero denominators; sign-change bisection alone is insufficient.",
    }

    reports["root_not_in_sqrtsum"] = {
        "affine_points": {"A": ["0", "0"], "B": ["w", "1"], "C": ["1+sqrt(2)", "w"]},
        "orientation": "w^2-1-sqrt(2)",
        "positive_root": "sqrt(1+sqrt(2))",
        "minimal_polynomial": "x^4-2*x^2-1",
        "reason": "Irreducible quartic has two non-real conjugates. Every rational linear combination of square roots of positive rationals lies in a totally real multiquadratic extension, so the root cannot be a SqrtSumV1.",
        "alternative": "Polynomial over coefficient field + isolating rational interval + root index, or named unsupported-root refusal.",
    }

    radical = SqrtSumV1(((2, Fraction(1)), (3, Fraction(1))))
    widths = [Fraction(math.isqrt(2 << (2 * bits)) + math.isqrt(3 << (2 * bits)), 1 << bits) for bits in (32, 80)]
    budget_rows = []
    for width in widths:
        budget = exact_work_budget(stage="PROOF_REVIEW", cap=1)
        error = None
        sign = None
        try:
            sign = (radical - SqrtSumV1.rational(width)).sign(budget=budget)
        except Exception as exc:
            error = {"type": type(exc).__name__, "detail": str(exc)}
        budget_rows.append({"width": frac(width), "float_width": float(width), "sign": sign, "spent": budget.spent, "error": error})
    assert budget_rows[0]["sign"] == 1 and budget_rows[0]["spent"] == 0
    assert budget_rows[1]["error"]["type"] == "ExactCanonicalizationWorkBudgetExhausted"
    # Both approximants are strictly below sqrt(2)+sqrt(3): they sum integer floor roots.
    reports["half_cap_counterexample_real_code"] = {
        "predicate": "sqrt(2)+sqrt(3)-w > 0",
        "true_sign_both": 1,
        "cap": 1,
        "base_spent_below_half_cap": True,
        "width_difference": frac(widths[1] - widths[0]),
        "width_difference_float": float(widths[1] - widths[0]),
        "rows": budget_rows,
        "scope": "Synthetic small cap disproves a generic margin theorem; no claim that either field scene fails its production cap.",
    }

    def ordered_merges(ties):
        groups = {frozenset(("A",)), frozenset(("B", "C")), frozenset(("D",))}
        for a, b in ties:
            ga = next(g for g in groups if a in g)
            gb = next(g for g in groups if b in g)
            merged = ga | gb
            if ga != gb and len(merged) <= 3:
                groups.remove(ga)
                groups.remove(gb)
                groups.add(merged)
        return sorted("".join(sorted(g)) for g in groups)
    order_ab_cd = ordered_merges([("A", "B"), ("C", "D")])
    order_cd_ab = ordered_merges([("C", "D"), ("A", "B")])
    assert order_ab_cd != order_cd_ab
    reports["initial_disjoint_faces_not_enough"] = {
        "initial_tied_pairs": ["A/B", "C/D"],
        "earlier_merge": "B/C",
        "toy_union_admission": "at most 3 original faces (stand-in for a union-dependent predicate)",
        "order_A_B_then_C_D": order_ab_cd,
        "reverse_order": order_cd_ab,
        "scope": "Abstract counterexample to using initial face disjointness; not a CFTUV field failure.",
        "sufficient_route": "Check current mutable read/write support after all preceding merges, include complete tie equivalence group and global side effects; prove equality modulo permitted internal IDs.",
    }

    target = ARGS.output.resolve()
    target.write_text(json.dumps(reports, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({"output": str(target), "probe_groups": len(reports), "all_assertions": "PASS", "budget_width_delta": float(widths[1] - widths[0])}, ensure_ascii=False))


if __name__ == "__main__":
    main()
