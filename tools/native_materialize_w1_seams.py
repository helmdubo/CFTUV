"""Part seams of W1 (`mesh._contour_polygons`, `mesh._fan_faces`): generated contours, UV maps and flags, the oracle against the native seam, on a cold memory.

Whole operations (`native_materialize_w1_corpus.py`) reach the geometry only as far as the field and the kernel tests build it. A contour made here is a random polygon on a grid (a star
polygon, so mostly simple; some are twisted on purpose), its coordinates rational or with a square root, its `(s, r)` an affine map of the chart position with a kink, a fold or nothing; the
flags (`in_flow`, `partition`, `exact_plane`, `reverse`, `affine_known`) are drawn. Contours of 3..12 vertices cover both planners of the partition law (exact up to 8 vertices, greedy above)
and every refusal that a piece can meet. The answer compared is everything the oracle leaves: the polygons, the tally (names, order, values), the exception with its text, the six articles, the sign
counters, the four memory tables with their order.

    python tools/native_materialize_w1_seams.py [--cases 300] [--seed 1]
"""

from __future__ import annotations

import argparse
import random
import sys
from collections import Counter
from fractions import Fraction
from pathlib import Path
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "kernel" / "src"))
sys.path.insert(0, str(ROOT / "tools"))

import native_corpus as nc  # noqa: E402
from cftuv_envelope import exact_sqrt_sum as exact  # noqa: E402
from cftuv_envelope.exact_sqrt_sum import SqrtSumV1  # noqa: E402
from cftuv_envelope.materialize import mesh  # noqa: E402
from cftuv_envelope.wavefront.faces import doubled_shoelace  # noqa: E402


def number(rng: random.Random, grid: int, roots: bool):
    """A coordinate: a small rational, or `a + b sqrt(k)` with a small `k`."""

    base = SqrtSumV1.rational(Fraction(rng.randint(-grid, grid), rng.choice((1, 1, 1, 2, 3))))
    if roots and rng.random() < 0.35:
        return base + SqrtSumV1.radical(Fraction(rng.randint(1, 3), rng.choice((1, 2))), rng.choice((2, 3, 5, 6)))
    return base


def star(rng: random.Random, size: int, grid: int, roots: bool) -> list:
    """`size` points around a centre, in angular order with distinct radii: a star polygon (simple), or twisted when the draw says so."""

    import math

    angles = sorted(rng.uniform(0, 2 * math.pi) for _ in range(size))
    points = []
    for angle in angles:
        radius = rng.randint(2, grid)
        x, y = round(radius * math.cos(angle)), round(radius * math.sin(angle))
        points.append((SqrtSumV1.rational(x) + (number(rng, 1, roots) if roots and rng.random() < 0.3 else SqrtSumV1.zero()), SqrtSumV1.rational(y)))
    if rng.random() < 0.12 and size >= 4:
        a, b = rng.sample(range(size), 2)
        points[a], points[b] = points[b], points[a]
    return points


def uv_map(rng: random.Random, points: list, roots: bool) -> list:
    """`(s, r)` of each point: an affine map of the position with a kink at a vertex, a fold, or exactly affine."""

    ax, ay = Fraction(rng.randint(1, 3), rng.choice((1, 2))), Fraction(rng.randint(-2, 2), rng.choice((1, 3)))
    bx, by = Fraction(rng.randint(-2, 2)), Fraction(rng.randint(1, 3))
    values = []
    for x, y in points:
        values.append((x.scaled(ax) + y.scaled(ay), x.scaled(bx) + y.scaled(by)))
    draw = rng.random()
    if draw < 0.7 and values:
        for _ in range(rng.randint(1, 2)):
            at = rng.randrange(len(values))
            bump = SqrtSumV1.rational(Fraction(rng.randint(1, 4), rng.choice((2, 3)))) + (SqrtSumV1.radical(1, rng.choice((2, 3))) if roots and rng.random() < 0.4 else SqrtSumV1.zero())
            values[at] = (values[at][0], values[at][1] + bump)
    if 0.7 <= draw < 0.85 and len(values) >= 4:
        at = rng.randrange(len(values))
        values[at] = (values[at][0], values[at][1] * SqrtSumV1.rational(-1))
    return values


def cases(seed: int, count: int):
    """`count` cases `{kind, cycle, uv, piece, flags}`; the same seed, the same cases."""

    rng = random.Random(seed)
    for number_ in range(count):
        size = rng.choice((3, 4, 4, 5, 5, 6, 7, 8, 8, 9, 10, 12))
        roots = rng.random() < 0.4
        points = star(rng, size, rng.choice((3, 4, 6)), roots)
        values = uv_map(rng, points, roots)
        keys = [f"v{index}" for index in range(size)]
        if size >= 4 and rng.random() < 0.08:
            keys[rng.randrange(size)] = keys[0]  # a repeated key: the contour touches itself
        cycle = list(zip(keys, (tuple(point) for point in points)))
        uv = {key: value for key, value in zip(keys, values)}
        area = doubled_shoelace(tuple(points))
        if rng.random() < 0.06:
            area = area + SqrtSumV1.rational(1)  # the area the contour must close is wrong
        kind = rng.choice(("contour", "contour", "contour", "fan"))
        owner = (rng.randint(-3, 3), rng.randint(-3, 3), 1, 1) if kind == "contour" else (0, 0, 0, 0)
        if kind == "fan":
            apex = rng.randrange(size)
            node = [coordinate.as_rational() for coordinate in points[apex]]
            owner = (int(node[0]), int(node[1]), 0, 0) if all(value is not None and value.denominator == 1 for value in node) else (10**6, 10**6, 0, 0)
        yield {
            "id": f"{seed}:{number_}",
            "kind": kind,
            "cycle": cycle,
            "uv": uv,
            "area": area,
            "owner": owner,
            "reverse": rng.random() < 0.5,
            "exact_plane": rng.random() < 0.8,
            "in_flow": rng.random() < 0.6,
            "partition": rng.random() < 0.8,
            "affine_known": rng.random() < 0.1,
            "unit": Fraction(rng.choice((1, 2, 4)), rng.choice((1, 3))),
        }


def run(case: dict, native: bool):
    """`(result, exception, state after)` of one case from a cold memory and a fresh budget; `native` runs the port."""

    exact.reset_factorization_memory()
    exact.reset_sign_counts()
    exact.reset_unbudgeted_work()
    budget = exact.exact_work_budget(stage="MATERIALIZE", domain_id="seam")
    before = nc.capture_state(budget, None)
    tally: Counter = Counter()
    piece = SimpleNamespace(owner=case["owner"], doubled_area=case["area"])
    frame_face = SimpleNamespace(face=piece, is_fan=True)
    if native:
        import cftuv_native

        mirror = cftuv_native.default_mirror()
        mirror.invalidate()
        try:
            if case["kind"] == "contour":
                result = mirror.w1_contour_polygons(piece, case["cycle"], budget, case["reverse"], case["exact_plane"], tally, case["uv"], case["unit"], case["in_flow"], case["partition"], case["affine_known"])
            else:
                result = mirror.w1_fan_faces(frame_face, case["cycle"], budget, case["reverse"], case["exact_plane"], tally, case["uv"], case["affine_known"])
            error = None
        except cftuv_native.NATIVE_REFUSALS as refusal:
            result, error = None, ("PORT", type(refusal).__qualname__, str(refusal))
        except Exception as exception:  # noqa: BLE001 - an exception of the oracle is its outcome
            result, error = None, nc.exception_info(exception)
    else:
        try:
            if case["kind"] == "contour":
                result = mesh._contour_polygons(piece, case["cycle"], budget, case["reverse"], case["exact_plane"], tally, case["uv"].__getitem__, case["unit"], case["in_flow"], case["partition"], case["affine_known"])
            else:
                result = mesh._fan_faces(frame_face, case["cycle"], budget, case["reverse"], case["exact_plane"], tally, case["uv"].__getitem__, case["affine_known"])
            error = None
        except Exception as exception:  # noqa: BLE001
            result, error = None, nc.exception_info(exception)
    return result, error, (type(tally).__qualname__, tuple(tally.items())), before, nc.capture_state(budget, None)


def compare(case: dict):
    """Everything the two sides disagree on (empty: equal), and whether the port refused."""

    expected = run(case, False)
    actual = run(case, True)
    if actual[1] is not None and actual[1][0] == "PORT":
        return [], actual[1]
    differences = []
    if nc.canonical(expected[0]) != nc.canonical(actual[0]):
        differences.append(f"result {expected[0]!r} != {actual[0]!r}")
    if expected[1] != actual[1]:
        differences.append(f"exception {expected[1]!r} != {actual[1]!r}")
    if nc.canonical(expected[2]) != nc.canonical(actual[2]):
        differences.append(f"tally {expected[2]!r} != {actual[2]!r}")
    differences += [str(item) for item in nc._compare_budget(expected[3], expected[4], actual[4])]
    differences += [str(item) for item in nc._compare_counters(expected[3], expected[4], actual[4])]
    differences += [str(item) for item in nc._compare_tables(expected[4], actual[4])]
    return differences, None


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--cases", type=int, default=300)
    parser.add_argument("--seed", type=int, default=1)
    arguments = parser.parse_args(argv)
    tally: Counter = Counter()
    failed = []
    for case in cases(arguments.seed, arguments.cases):
        differences, refusal = compare(case)
        tally["cases"] += 1
        tally["refused by the port"] += refusal is not None
        if differences:
            failed.append((case["id"], differences[:2]))
        else:
            tally["equal"] += 1
        expected = run(case, False)
        tally["oracle raised"] += expected[1] is not None
        for name, _value in expected[2][1]:
            tally[name] += 1
    print(dict(tally))
    for case_id, differences in failed[:10]:
        print("DIFFERENT", case_id, differences)
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
