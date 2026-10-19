#!/usr/bin/env python3
"""Independent exact LSQ-event check, standard library only.

CLI: --input independent_focus.json --event exact_event.json --output result.json
Conditional on the recorded affine operands. No field replay or binary64
switch-threshold claim. Exact arithmetic in Q(sqrt(a), sqrt(b)); interval
Horner with rational bounds for the two positive radicals.
"""
from __future__ import annotations

import argparse
import ast
from decimal import Decimal, localcontext
from fractions import Fraction as F
import hashlib
from math import isqrt
from pathlib import Path
import json


def main():
    cli = argparse.ArgumentParser()
    cli.add_argument("--input", required=True)
    cli.add_argument("--event", required=True)
    cli.add_argument("--output", required=True)
    cli.add_argument("--bits", default=320, type=int)
    args = cli.parse_args()
    data_bytes = Path(args.input).read_bytes()
    event_bytes = Path(args.event).read_bytes()
    data, event = json.loads(data_bytes), json.loads(event_bytes)
    radicands = sorted({int(q) for part in data["coefficients"].values() for point in part for axis in point for kind in ("intercept", "slope") for q, _ in axis[kind] if int(q) != 1})
    assert len(radicands) == 2, radicands
    a, b = radicands
    assert all(q > 0 and isqrt(q) ** 2 != q for q in (a, b, a * b))
    assert radicands == event["radicands"]
    basis = (1, a, b, a * b)
    zero = (F(0), F(0), F(0), F(0))

    def scalar(v):
        return (F(v), F(0), F(0), F(0))

    one = scalar(1)

    def add(x, y):
        return tuple(c + d for c, d in zip(x, y))

    def neg(x):
        return tuple(-c for c in x)

    def mul(x, y):
        result = [F(0)] * 4
        for i, c in enumerate(x):
            if not c:
                continue
            for j, d in enumerate(y):
                if not d:
                    continue
                factor = (a if i & j & 1 else 1) * (b if i & j & 2 else 1)
                result[i ^ j] += c * d * factor
        return tuple(result)

    def scale(x, v):
        return tuple(c * v for c in x)

    def root(q):
        q = int(q)
        assert q > 0
        # Avoid integer factorization: sqrt(q)/sqrt(basis[k]) is rational
        # exactly when numerator and denominator of the reduced ratio square.
        for k, divisor in enumerate(basis):
            ratio = F(q, divisor)
            n, d = isqrt(ratio.numerator), isqrt(ratio.denominator)
            if n * n == ratio.numerator and d * d == ratio.denominator:
                coefficients = [F(0)] * 4
                coefficients[k] = F(n, d)
                return tuple(coefficients)
        raise ValueError("radical is outside the recorded biquadratic basis: " + str(q))

    def terms(entries):
        result = zero
        for q, coefficient in entries:
            result = add(result, scale(root(q), F(coefficient)))
        return result

    def trim(p):
        p = list(p)
        while len(p) > 1 and p[-1] == zero:
            p.pop()
        return tuple(p)

    def padd(p, q):
        n = max(len(p), len(q))
        return trim(add(p[i] if i < len(p) else zero, q[i] if i < len(q) else zero) for i in range(n))

    def pneg(p):
        return tuple(neg(c) for c in p)

    def psub(p, q):
        return padd(p, pneg(q))

    def pmul(p, q):
        result = [zero] * (len(p) + len(q) - 1)
        for i, c in enumerate(p):
            if c != zero:
                for j, d in enumerate(q):
                    if d != zero:
                        result[i + j] = add(result[i + j], mul(c, d))
        return trim(result)

    def pscale(p, v):
        return trim(scale(c, v) for c in p)

    def psum(items):
        result = (zero,)
        for p in items:
            result = padd(result, p)
        return result

    def axis(record):
        return trim((terms(record["intercept"]), terms(record["slope"])))

    points = [[axis(v) for v in point] for point in data["coefficients"]["points"]]
    facts = [[axis(v) for v in point] for point in data["coefficients"]["facts"]]
    count = len(points)
    assert count == len(facts) == len(data["union"]) and count >= 3
    centres = [pscale(psum(point[j] for point in points), F(1, count)) for j in (0, 1)]
    x = [psub(point[0], centres[0]) for point in points]
    y = [psub(point[1], centres[1]) for point in points]
    sxx = psum(pmul(v, v) for v in x)
    sxy = psum(pmul(v, w) for v, w in zip(x, y))
    syy = psum(pmul(v, v) for v in y)
    determinant = psub(pmul(sxx, syy), pmul(sxy, sxy))
    residuals = []
    for j in (0, 1):
        mean = pscale(psum(f[j] for f in facts), F(1, count))
        centred = [psub(f[j], mean) for f in facts]
        rx = psum(pmul(v, f) for v, f in zip(x, centred))
        ry = psum(pmul(v, f) for v, f in zip(y, centred))
        nx = psub(pmul(rx, syy), pmul(ry, sxy))
        ny = psub(pmul(sxx, ry), pmul(sxy, rx))
        residuals.append([psub(psub(pmul(f, determinant), pmul(nx, v)), pmul(ny, w)) for f, v, w in zip(centred, x, y)])
    width_poly = (zero, one)
    allowance = pscale(pmul(determinant, width_poly), F(data["lattice_scale"]) / 256)
    allowance_square = pmul(allowance, allowance)
    polynomials = [psub(padd(pmul(u, u), pmul(v, v)), allowance_square) for u, v in zip(*residuals)]
    active = int(event["active_vertex_index"])
    assert 0 <= active < count
    assert event["active_vertex"] == data["union"][active]

    # Whitelisted parser, not eval: integers, arithmetic, rational division,
    # and sqrt(integer). The published coefficient expression is untrusted.
    def parse(expression):
        tree = ast.parse(expression, mode="eval")
        if len(list(ast.walk(tree))) > 10000:
            raise ValueError("expression too large")

        def walk(node):
            if isinstance(node, ast.Constant) and type(node.value) is int:
                return scalar(node.value)
            if isinstance(node, ast.UnaryOp) and isinstance(node.op, (ast.USub, ast.UAdd)):
                value = walk(node.operand)
                return neg(value) if isinstance(node.op, ast.USub) else value
            if isinstance(node, ast.BinOp):
                left, right = walk(node.left), walk(node.right)
                if isinstance(node.op, ast.Add):
                    return add(left, right)
                if isinstance(node.op, ast.Sub):
                    return add(left, neg(right))
                if isinstance(node.op, ast.Mult):
                    return mul(left, right)
                if isinstance(node.op, ast.Div) and right[1:] == zero[1:] and right[0]:
                    return scale(left, 1 / right[0])
            if isinstance(node, ast.Call) and isinstance(node.func, ast.Name) and node.func.id == "sqrt" and len(node.args) == 1 and not node.keywords:
                argument = walk(node.args[0])
                if argument[1:] == zero[1:] and argument[0].denominator == 1:
                    return root(argument[0].numerator)
            raise ValueError("unsupported coefficient AST: " + type(node).__name__)

        return walk(tree.body)

    recorded = trim(parse(c) for c in event["polynomial_coefficients_low_to_high"])
    polynomial_match = recorded == polynomials[active]
    assert polynomial_match, "reconstructed polynomial differs from published coefficients"
    assert len(polynomials[active]) - 1 == event["polynomial_degree"] == 10

    def iadd(left, right):
        return (left[0] + right[0], left[1] + right[1])

    def imul(left, right):
        values = [x * y for x in left for y in right]
        return min(values), max(values)

    scale_bits = 1 << args.bits

    def radical_interval(q):
        low = isqrt(q << (2 * args.bits))
        high = low if low * low == q << (2 * args.bits) else low + 1
        return F(low, scale_bits), F(high, scale_bits)

    ia, ib = radical_interval(a), radical_interval(b)
    bases = ((F(1), F(1)), ia, ib, imul(ia, ib))
    coefficient_cache = {}

    def enclose(coefficient):
        if coefficient in coefficient_cache:
            return coefficient_cache[coefficient]
        result = (F(0), F(0))
        for c, interval in zip(coefficient, bases):
            result = iadd(result, imul((c, c), interval))
        coefficient_cache[coefficient] = result
        return result

    def horner(polynomial, low, high):
        result = (F(0), F(0))
        for coefficient in reversed(polynomial):
            result = iadd(imul(result, (low, high)), enclose(coefficient))
        return result

    low, high = map(F, event["isolating_rationals"])
    assert 0 < low < high
    chosen = polynomials[active]
    left = horner(chosen, low, low)
    right = horner(chosen, high, high)
    derivative = tuple(scale(coefficient, degree) for degree, coefficient in enumerate(chosen) if degree)
    derivative_range = horner(derivative, low, high)
    determinant_range = horner(determinant, low, high)
    other_ranges = {str(i): horner(p, low, high) for i, p in enumerate(polynomials) if i != active}
    checks = {
        "published_input_sha256_matches": hashlib.sha256(data_bytes).hexdigest() == event["input_sha256"],
        "field_degree_is_four": all(isqrt(q) ** 2 != q for q in (a, b, a * b)),
        "recorded_polynomial_matches_reconstruction": polynomial_match,
        "left_sign_positive": left[0] > 0,
        "right_sign_negative": right[1] < 0,
        "derivative_negative_on_isolator": derivative_range[1] < 0,
        "determinant_positive_on_isolator": determinant_range[0] > 0,
        "other_vertex_margins_negative_on_isolator": all(v[1] < 0 for v in other_ranges.values()),
    }
    with localcontext() as ctx:
        ctx.prec = 70
        middle = (low + high) / 2
        root_decimal = str(Decimal(middle.numerator) / Decimal(middle.denominator))

    def interval_record(pair):
        return [str(v) for v in pair]

    result = {
        "verdict": "VERIFIED_CONDITIONAL_IDEAL_LSQ_ROOT" if all(checks.values()) else "FAILED",
        "checks": checks,
        "input_sha256": hashlib.sha256(data_bytes).hexdigest(),
        "event_sha256": hashlib.sha256(event_bytes).hexdigest(),
        "recorded_source_base": data["base"],
        "arithmetic": "stdlib Fraction field Q(sqrt(a),sqrt(b)); reconstructed polynomial; rational interval Horner",
        "radicands": radicands,
        "basis": ["1", "sqrt(a)", "sqrt(b)", "sqrt(a)*sqrt(b)"],
        "interval_bits": args.bits,
        "active_vertex_index": active,
        "active_vertex": data["union"][active],
        "polynomial_degree": len(chosen) - 1,
        "obvious_zero_root_multiplicity": next(i for i, c in enumerate(chosen) if c != zero),
        "degree_after_removing_obvious_width_factor": len(chosen) - 1 - next(i for i, c in enumerate(chosen) if c != zero),
        "minimal_polynomial_caveat": "The cleared-denominator degree is not a proved minimal degree of the isolated root; no irreducibility or exclusion from a smaller number field is claimed.",
        "determinant_degree": len(determinant) - 1,
        "isolating_rationals": [str(low), str(high)],
        "midpoint_decimal_for_display": root_decimal,
        "proof_intervals": {"left_value": interval_record(left), "right_value": interval_record(right), "derivative": interval_record(derivative_range), "determinant": interval_record(determinant_range), "other_vertex_polynomials": {k: interval_record(v) for k, v in other_ranges.items()}},
        "scope": "Unique ideal real-arithmetic LSQ threshold inside the published isolator, conditional on recorded affine point/station operands. Not full field replay, not proof of affine laws throughout that interval, and not the binary64 switching width of the product.",
        "independence": "Reconstructs all six polynomials from points/facts without SymPy. Published polynomial is only compared afterwards using an arithmetic-only AST interpreter. Published signs/derivative flags are not used as inputs to the proof.",
    }
    Path(args.output).write_text(json.dumps(result, indent=2, ensure_ascii=False) + "\n")
    print(json.dumps({key: result[key] for key in ("verdict", "checks", "polynomial_degree", "midpoint_decimal_for_display")}, ensure_ascii=False))
    return 0 if all(checks.values()) else 1


if __name__ == "__main__":
    raise SystemExit(main())
