"""Точный аудит конечной таблицы стартовых значений legacy atan; runtime не меняет."""

from __future__ import annotations

import argparse
from fractions import Fraction as F
import json
import math
from pathlib import Path
from types import FunctionType, SimpleNamespace


def atan_bounds(x: F, terms: int = 128):
    """Соседние частичные суммы знакопеременного ряда при |x|<1."""
    assert abs(x) < 1
    total, power = F(0), x
    for index in range(terms):
        total += power / (2 * index + 1)
        power *= -x*x
    adjacent = total + power / (2 * terms + 1)
    return min(total, adjacent), max(total, adjacent)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--newton", action="store_true")
    args = parser.parse_args()
    a, b = atan_bounds(F(1, 5)), atan_bounds(F(1, 239))
    pi = (16*a[0]-4*b[1], 16*a[1]-4*b[0])
    records = []
    for n in range(256):
        if n <= 64:
            lower, upper = atan_bounds(F(n, 128))
        else:
            delta = atan_bounds(F(n-128, n+128))
            lower, upper = pi[0]/4+delta[0], pi[1]/4+delta[1]
        floor_low = (lower.numerator << 50) // lower.denominator
        floor_high = (upper.numerator << 50) // upper.denominator
        if floor_low != floor_high:
            raise AssertionError(f"insufficient seed proof at n={n}")
        # Только наблюдение старого источника, не решающий путь продукта.
        legacy = int(math.atan(n / 128) * 2**53) >> 3
        error_low, error_high = lower-F(legacy, 2**50), upper-F(legacy, 2**50)
        records.append({"n": n, "legacy_seed": legacy, "certified_floor_seed": floor_low,
                        "seed_difference": legacy-floor_low,
                        "legacy_error_strictly_below_2pow_minus48": max(abs(error_low), abs(error_high)) < F(1, 2**48)})
    result = {"scope": "n/128 lattice 0<=n<=255; not proof of Newton output or general high-precision atan",
              "records": records, "count": len(records),
              "different_from_exact_floor": sum(item["seed_difference"] != 0 for item in records),
              "all_seed_errors_below_2pow_minus48": all(item["legacy_error_strictly_below_2pow_minus48"] for item in records)}
    if args.newton:
        # Меняется только локальная копия globals одной функции оракула.
        from mpmath.libmp import libelefun
        comparisons, differences = 0, []
        for item in records:
            seed = item["certified_floor_seed"]
            environment = dict(libelefun.atan_newton.__globals__)
            environment["math"] = SimpleNamespace(atan=lambda _x, seed=seed: seed / 2**50)
            exact_seed_newton = FunctionType(libelefun.atan_newton.__code__, environment)
            for precision in (84, 148, 276, 532, 1044, 2068):
                x = item["n"] << (precision - 7)
                expected = libelefun.atan_newton(x, precision)
                actual = exact_seed_newton(x, precision)
                comparisons += 1
                if expected != actual:
                    differences.append({"n": item["n"], "precision": precision,
                                        "difference": int(actual-expected)})
        result["newton"] = {"cases": comparisons, "differences": differences,
                            "scope": "finite lattice and six cache precisions only"}
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(result, indent=2), encoding="utf-8")
    print(json.dumps({key: value for key, value in result.items() if key != "records"}), flush=True)


if __name__ == "__main__":
    main()
