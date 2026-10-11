"""Adversarial pairs for the native `effective_alpha` port: generated here, Blender-free, run through `boundary._contact_candidates` (the recorder of `native_effective_alpha_corpus` records them).

The call is made, not recorded, by this module: `generate(scale, seed)` runs every scenario against whatever `boundary._contact_candidates` is (the oracle, or the recorder's wrapper).
Scenarios (a scenario is a metric, a source segment and a list of barrier segments, run with and without the per-source frame, under the modes the scenario names):

  rational    random rational Gram matrices and grid segments (the product's case): with the prefilter and without, native / shadow / sympy
  degenerate  parallel, collinear, touching, zero-length barrier, zero-length source (the oracle's `ValueError`), equal-alpha ties, contacts at the endpoints
  irrational  irrational barrier / source coordinates (V2 texts and legacy sympy texts), irrational parameters (the port declines these pairs)
  outside     texts outside the native field (nested radical, cube root, pi): `OutsideNativeField` in the oracle, the sympy route
  classes3    barrier coordinates with two independent radicals (three square classes: the sign ladder of the algebra)
  huge        30+ digit numerators, denominators and radicands (beyond binary64: the prefilter yields)
  ties        a barrier parallel to the source with irrational stations: two parameters with EQUAL alpha (the order of tied contacts)
  roots       irrational METRICS (a gram with a non-square determinant: covectors in two square classes, the representatives of the classes decide the terms)

A pure function of `(scale, seed)`: the same arguments make the same calls in the same order (`PYTHONHASHSEED` plays no part: the oracle's sympy route is insertion-ordered).
"""

from __future__ import annotations

import random
import sys
from fractions import Fraction
from pathlib import Path
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[1]
KERNEL_SOURCE = ROOT / "kernel" / "src"
if str(KERNEL_SOURCE) not in sys.path:
    sys.path.insert(0, str(KERNEL_SOURCE))

import sympy as sp  # noqa: E402

from cftuv_envelope.reference import boundary as boundary_module  # noqa: E402
from cftuv_envelope.reference import native_exact as native_exact  # noqa: E402
from cftuv_envelope.reference import planar_types as planar_types  # noqa: E402
from cftuv_envelope.reference import symbolic_backend as symbolic  # noqa: E402
from cftuv_envelope.reference.boundary_native import SourceContactFrame  # noqa: E402
from cftuv_envelope.reference.metric import ExactPlanarMetric  # noqa: E402

DEFAULT_SEED = 20261011


def _rounds(base: int, scale) -> int:
    """How many rounds a scenario of `base` rounds makes at `scale` (a number: 1 is the full set, a fraction a smaller one; never fewer than one)."""

    return max(1, round(base * scale))


def metric_of(gram):
    matrix = sp.Matrix(gram)
    inverse = matrix.inv()
    return SimpleNamespace(
        metric=ExactPlanarMetric(
            tuple(tuple(sp.Rational(item) for item in row) for row in gram),
            tuple(tuple(sp.Rational(item) for item in row) for row in inverse.tolist()),
            1,
        )
    )


def point(x, y):
    return planar_types.ExactPlanarPoint(planar_types.ExactScalar(x), planar_types.ExactScalar(y))


def rat(value):
    return native_exact._rational_text(Fraction(value))


def source_of(context, start, end, support="s"):
    """A source as the compile makes it: the unit tangent and the owner normal through the metric (sympy forms for irrational lengths)."""

    first, last = point(rat(start[0]), rat(start[1])), point(rat(end[0]), rat(end[1]))
    tangent = context.metric.unit_g(planar_types.point_sub(last, first))
    normal = context.metric.owner_normal_g(tangent, owner_left=True)
    return SimpleNamespace(start=first, end=last, tangent=tangent, owner_normal=normal, support_id=support)


def boundary_of(a, b, segment_id="b"):
    return SimpleNamespace(segment=SimpleNamespace(start=point(*a), end=point(*b), segment_id=segment_id))


def R(x, y):
    return rat(x), rat(y)


def run_pairs(context, source, boundaries, modes=("NATIVE_EXACT",), with_frame=True):
    """Every barrier against the source under every mode; a pair that raises is a pair (the exception is the oracle's answer)."""

    for mode in modes:
        with symbolic.symbolic_backend(symbolic.SymbolicBackendV1(mode), symbolic.DisagreementPolicyV1.RECORD):
            frame = SourceContactFrame(context, source) if with_frame and mode != "SYMPY" else None
            for boundary in boundaries:
                try:
                    boundary_module._contact_candidates(context, source, boundary, frame)
                except Exception:  # noqa: BLE001 - recorded by the caller of the wrapper
                    pass


def random_gram(rng):
    while True:
        a = Fraction(rng.randint(1, 9), rng.randint(1, 4))
        c = Fraction(rng.randint(1, 9), rng.randint(1, 4))
        b = Fraction(rng.randint(-4, 4), rng.randint(1, 4))
        if a * c - b * b > 0:
            return [[a, b], [b, c]]


def grid_point(rng, span=8, denominators=(1, 1, 1, 2, 3, 5)):
    d = rng.choice(denominators)
    return Fraction(rng.randint(-span * d, span * d), d), Fraction(rng.randint(-span * d, span * d), d)


def scenario_rational(rng, scale):
    for _ in range(_rounds(40, scale)):
        context = metric_of(random_gram(rng))
        a, b = grid_point(rng), grid_point(rng)
        if a == b:
            continue
        source = source_of(context, a, b)
        boundaries = [boundary_of(R(*grid_point(rng)), R(*grid_point(rng)), f"b{index}") for index in range(rng.randint(8, 30))]
        run_pairs(context, source, boundaries, modes=("NATIVE_EXACT",), with_frame=True)
        run_pairs(context, source, boundaries[:6], modes=("NATIVE_EXACT", "SHADOW", "SYMPY"), with_frame=False)
        run_pairs(context, source, boundaries[:6], modes=("SHADOW",), with_frame=True)


def scenario_degenerate(rng, scale):
    for _ in range(_rounds(12, scale)):
        context = metric_of(random_gram(rng) if rng.random() < 0.7 else [[1, 0], [0, 1]])
        a, b = grid_point(rng, 6), grid_point(rng, 6)
        if a == b:
            b = (a[0] + 1, a[1])
        source = source_of(context, a, b)
        direction = (b[0] - a[0], b[1] - a[1])
        boundaries = []
        for t in (Fraction(0), Fraction(1, 3), Fraction(1), Fraction(2)):  # parallel and collinear
            p = (a[0] + direction[0] * t, a[1] + direction[1] * t)
            q = (p[0] + direction[0], p[1] + direction[1])
            boundaries.append(boundary_of(R(*p), R(*q), "col"))
            boundaries.append(boundary_of(R(p[0] + direction[1], p[1] - direction[0]), R(q[0] + direction[1], q[1] - direction[0]), "par"))
        for t in (Fraction(0), Fraction(1, 2), Fraction(1)):  # crossing through the source endpoints and the middle
            mid = (a[0] + direction[0] * t, a[1] + direction[1] * t)
            boundaries.append(boundary_of(R(mid[0] - direction[1], mid[1] + direction[0]), R(mid[0] + direction[1], mid[1] - direction[0]), "perp"))
            boundaries.append(boundary_of(R(mid[0] - 1, mid[1] - 1), R(mid[0] + 1, mid[1] + 1), "diag"))
        boundaries.append(boundary_of(R(*a), R(a[0] + 1, a[1] + 2), "touch"))
        boundaries.append(boundary_of(R(*b), R(b[0] - 3, b[1] + 1), "touch2"))
        boundaries.append(boundary_of(R(*a), R(*a), "zero"))
        boundaries.append(boundary_of(R(a[0] + 1, a[1] + 1), R(a[0] + 1, a[1] + 1), "zero2"))
        run_pairs(context, source, boundaries, modes=("NATIVE_EXACT", "SHADOW", "SYMPY"), with_frame=True)
        run_pairs(context, source, boundaries, modes=("NATIVE_EXACT",), with_frame=False)
    # a zero-length source: the `ValueError` of the metric length, with and without a frame
    context = metric_of([[1, 0], [0, 1]])
    start = point(rat(1), rat(2))
    zero = SimpleNamespace(
        start=start, end=start, tangent=planar_types.ExactPlanarVector(planar_types.ExactScalar(rat(1)), planar_types.ExactScalar(rat(0))),
        owner_normal=planar_types.ExactPlanarVector(planar_types.ExactScalar(rat(0)), planar_types.ExactScalar(rat(1))), support_id="zero-source",
    )
    run_pairs(context, zero, [boundary_of(R(0, 0), R(3, 3))], modes=("NATIVE_EXACT", "SHADOW", "SYMPY"), with_frame=True)
    run_pairs(context, zero, [boundary_of(R(0, 0), R(3, 3))], modes=("NATIVE_EXACT",), with_frame=False)


IRRATIONAL_TEXTS = (
    "Sqrt(Rational(2, 1))", "-Sqrt(Rational(2, 1))", "Sqrt(Rational(3, 4))", "-Sqrt(Rational(5, 9))", "Sqrt(Rational(8, 1))",
    "Sqrt(Rational(1, 2))", "Sqrt(Rational(12, 25))", "Sqrt(Rational(18, 1))",
    "Mul(Rational(1, 2), Pow(Integer(3), Rational(1, 2)))", "Mul(Integer(-1), Pow(Integer(2), Rational(1, 2)))",
    "Pow(Integer(5), Rational(1, 2))", "Mul(Rational(3, 7), Pow(Integer(2), Rational(1, 2)))",
)
OUTSIDE_TEXTS = (
    "Pow(Add(Integer(1), Pow(Integer(2), Rational(1, 2))), Rational(1, 2))", "Pow(Integer(2), Rational(1, 3))", "pi",
    "Mul(Integer(2), Pow(Add(Integer(3), Pow(Integer(5), Rational(1, 2))), Rational(1, 2)))",
)
CLASSES3_TEXTS = (
    "Add(Pow(Integer(2), Rational(1, 2)), Pow(Integer(3), Rational(1, 2)))",
    "Add(Integer(1), Pow(Integer(2), Rational(1, 2)), Pow(Integer(3), Rational(1, 2)))",
    "Add(Rational(1, 2), Mul(Rational(1, 3), Pow(Integer(5), Rational(1, 2))), Pow(Integer(7), Rational(1, 2)))",
    "Add(Mul(Integer(-1), Pow(Integer(2), Rational(1, 2))), Pow(Integer(3), Rational(1, 2)), Pow(Integer(5), Rational(1, 2)))",
)


def scenario_text(rng, scale, texts, rounds, modes=("NATIVE_EXACT", "SHADOW")):
    for _ in range(_rounds(rounds, scale)):
        context = metric_of(random_gram(rng) if rng.random() < 0.6 else [[1, 0], [0, 1]])
        a, b = grid_point(rng, 5), grid_point(rng, 5)
        if a == b:
            b = (a[0] + 2, a[1] + 1)
        source = source_of(context, a, b)
        boundaries = []
        for index in range(10):
            def coordinate():
                return rng.choice(texts) if rng.random() < 0.45 else rat(grid_point(rng)[0])
            boundaries.append(boundary_of((coordinate(), coordinate()), (coordinate(), coordinate()), f"t{index}"))
        run_pairs(context, source, boundaries, modes=modes, with_frame=True)
        run_pairs(context, source, boundaries[:4], modes=("NATIVE_EXACT",), with_frame=False)
        # an irrational SOURCE start as well
        irrational_source = SimpleNamespace(
            start=point(rng.choice(texts), rat(grid_point(rng)[1])), end=source.end, tangent=source.tangent, owner_normal=source.owner_normal, support_id="irr-source",
        )
        run_pairs(context, irrational_source, boundaries[:5], modes=("NATIVE_EXACT",), with_frame=True)


def scenario_huge(rng, scale):
    for _ in range(_rounds(6, scale)):
        context = metric_of(random_gram(rng))
        big = 10 ** rng.randint(18, 40)
        a = (Fraction(rng.randint(-big, big), rng.randint(1, big)), Fraction(rng.randint(-big, big), rng.randint(1, 7)))
        b = (a[0] + rng.randint(1, 9), a[1] - rng.randint(1, 9))
        source = source_of(context, a, b)
        boundaries = []
        for index in range(10):
            p = (a[0] + Fraction(rng.randint(-big, big), rng.randint(1, big)), a[1] + Fraction(rng.randint(-big, big), rng.randint(1, big)))
            q = (a[0] + Fraction(rng.randint(-5, 5), 1), a[1] + Fraction(rng.randint(-5, 5), 1))
            boundaries.append(boundary_of(R(*p), R(*q), f"h{index}"))
        run_pairs(context, source, boundaries, modes=("NATIVE_EXACT", "SHADOW"), with_frame=True)
    context = metric_of([[Fraction(10 ** 30 + 7, 3), 0], [0, Fraction(1, 10 ** 25 + 3)]])
    source = source_of(context, (Fraction(0), Fraction(0)), (Fraction(5), Fraction(3)))
    boundaries = [boundary_of(R(1, -2), R(4, 9), "g1"), boundary_of(R(-3, 1), R(6, 1), "g2")]
    run_pairs(context, source, boundaries, modes=("NATIVE_EXACT", "SHADOW"), with_frame=True)


def scenario_ties(rng, scale):
    """A barrier parallel to the source with irrational stations: two parameters with EQUAL alpha (the order of the tied contacts)."""

    for _ in range(_rounds(30, scale)):
        context = metric_of([[1, 0], [0, 1]])
        length = rng.randint(2, 6)
        vertical = rng.random() < 0.3
        a = (Fraction(0), Fraction(0))
        b = (Fraction(0), Fraction(length)) if vertical else (Fraction(length), Fraction(0))
        source = source_of(context, a, b)
        root = sp.sqrt(rng.choice((2, 3, 5, 6, 7, 10)))
        shift = Fraction(rng.randint(-8, 8), rng.randint(1, 4))
        coefficient = Fraction(rng.choice((1, 1, 2, 3)), rng.choice((1, 1, 2)))
        span = Fraction(length + rng.randint(2, 8), 1)
        height = Fraction(rng.randint(-4, 4), rng.randint(1, 3))
        start_along = sp.Rational(shift.numerator, shift.denominator) + sp.Rational(coefficient.numerator, coefficient.denominator) * root
        end_along = start_along + sp.Rational(span.numerator, span.denominator)
        across = rat(height)

        def text(value):
            return planar_types.ExactScalar.from_value(value).expression

        first, second = ((across, text(start_along)), (across, text(end_along))) if vertical else ((text(start_along), across), (text(end_along), across))
        boundaries = [boundary_of(first, second, "tie"), boundary_of(second, first, "tie-reversed")]
        run_pairs(context, source, boundaries, modes=("NATIVE_EXACT", "SYMPY", "SHADOW"), with_frame=True)
        run_pairs(context, source, boundaries, modes=("NATIVE_EXACT", "SYMPY", "SHADOW"), with_frame=False)


def scenario_roots(rng, scale):
    """Irrational metrics: the covectors of the source live in two square classes (1 and a root), so the representatives of the classes decide the terms of every alpha and station.

    The segments have rational ends, the parameters are ratios of elements of Q(sqrt(m)) that are often irrational (the port declines those contacts) and often rational.
    """

    for _ in range(_rounds(14, scale)):
        # a Gram matrix whose determinant is not a square, with entries that make the unit tangent irrational
        while True:
            a = Fraction(rng.randint(1, 7), rng.randint(1, 3))
            c = Fraction(rng.randint(1, 7), rng.randint(1, 3))
            b = Fraction(rng.randint(-3, 3), rng.randint(1, 3))
            if a * c - b * b > 0:
                break
        context = metric_of([[a, b], [b, c]])
        p, q = grid_point(rng, 4), grid_point(rng, 4)
        if p == q:
            q = (p[0] + 1, p[1] + 2)
        source = source_of(context, p, q)
        boundaries = [boundary_of(R(*grid_point(rng, 6)), R(*grid_point(rng, 6)), f"r{index}") for index in range(18)]
        run_pairs(context, source, boundaries, modes=("NATIVE_EXACT", "SHADOW"), with_frame=True)
        run_pairs(context, source, boundaries[:6], modes=("NATIVE_EXACT",), with_frame=False)


SCENARIOS = {
    "rational": scenario_rational,
    "degenerate": scenario_degenerate,
    "irrational": lambda rng, scale: scenario_text(rng, scale, IRRATIONAL_TEXTS, 14),
    "outside": lambda rng, scale: scenario_text(rng, scale, OUTSIDE_TEXTS, 6, modes=("NATIVE_EXACT",)),
    "classes3": lambda rng, scale: scenario_text(rng, scale, CLASSES3_TEXTS, 8),
    "huge": scenario_huge,
    "ties": scenario_ties,
    "roots": scenario_roots,
}


def generate(scale: int = 1, seed: int = DEFAULT_SEED, only: tuple = (), on_scenario=None) -> None:
    """Runs the scenarios (or those named in `only`) against the current `boundary._contact_candidates`; `on_scenario(name)` is called before each."""

    rng = random.Random(seed)
    for name, function in SCENARIOS.items():
        if only and name not in only:
            continue
        if on_scenario is not None:
            on_scenario(name)
        function(rng, scale)
