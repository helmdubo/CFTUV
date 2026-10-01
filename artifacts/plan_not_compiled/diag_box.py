"""Почему `_termination_boxes` не находит коробку: какой пункт `_box_is_feasible` падает.

  PYTHONSAFEPATH=1 PYTHONPATH=kernel/src python artifacts/plan_not_compiled/diag_box.py <dir> <patch> [density]

Перехватывает вход `_termination_boxes`, затем сам (копией логики
`_box_is_feasible`, но с причиной) прогоняет делители 2^24, 2^25 ... и на каждом
печатает ПЕРВЫЙ упавший пункт и превышение подшага над Delta_max в градусах
(float, только для чтения). Число итераций уточнения и израсходованные
order-steps — из реального цикла (`_BOX_REFINEMENT_CAP`).
"""
from __future__ import annotations

import json
import math
import sys
from fractions import Fraction
from pathlib import Path

import sympy as sp

from cftuv_envelope import AnalysisSnapshotCodecV1, DecalRequestCodecV1
from cftuv_envelope.reference import adaptive_density_fan as fan
from cftuv_envelope.reference.compile import compile_reference_envelopes

CAPTURED = []
CALLS = {"feasible": 0}


def _wrap():
    original_terminate = fan._termination_boxes
    original_feasible = fan._box_is_feasible

    def terminate(metric, ideal, orientation, q, records):
        CAPTURED.append((metric, ideal, orientation, q, records))
        return original_terminate(metric, ideal, orientation, q, records)

    def feasible(*args, **kwargs):
        CALLS["feasible"] += 1
        return original_feasible(*args, **kwargs)

    fan._termination_boxes = terminate
    fan._box_is_feasible = feasible


def angle_deg(metric, left, right):
    dot = fan._dual_dot(metric, left, right)
    norm = fan._dual_dot(metric, left, left) * fan._dual_dot(metric, right, right)
    return math.degrees(math.atan2(float(sp.N(sp.sqrt(sp.simplify(norm - dot * dot)), 40)), float(sp.N(dot, 40))))


def reason(metric, ideal, orientation, q, boxes):
    expected = fan._expected_orientation(orientation)
    endpoints = []
    for ordinal, (lower, upper, use_x, sign) in enumerate(boxes, start=1):
        pair = (
            fan._candidate_vector(lower, use_x, sign, metric),
            fan._candidate_vector(upper, use_x, sign, metric),
        )
        for item in pair:
            if not fan._inside_ordinal(metric, item, ideal[ordinal - 1], ideal[ordinal], ideal[ordinal + 1]):
                return f"endpoint outside ordinal window {ordinal}", None
        endpoints.append(pair)
    sequences = [()]
    for pair in endpoints:
        sequences = [(*prefix, item) for prefix in sequences for item in pair]
    worst = None
    for hidden in sequences:
        sequence = (ideal[0], *hidden, ideal[-1])
        for left, right in zip(sequence, sequence[1:]):
            if fan._sign(fan._oriented_cross(metric, left, right), metric) != expected:
                return "orientation of a step flips", None
            if not fan._subturn(metric, left, right, q):
                excess = angle_deg(metric, left, right) - 180.0 / q
                worst = excess if worst is None else max(worst, excess)
    if worst is not None:
        return "step exceeds Delta_max (subturn clause)", worst
    return "FEASIBLE", None


def main():
    root = Path(sys.argv[1])
    patch = sys.argv[2]
    density = int(sys.argv[3]) if len(sys.argv) > 3 else 4
    _wrap()
    base = root / f"patch_{patch}"
    snapshot = AnalysisSnapshotCodecV1.loads((base / "analysis_snapshot.json").read_bytes())
    request = DecalRequestCodecV1.loads((base / f"decal_request_d{density}.json").read_bytes())
    result = compile_reference_envelopes(snapshot, request)
    print("outcome", result.outcome.value, "| _box_is_feasible calls (refinements run):", CALLS["feasible"], "| cap:", fan._BOX_REFINEMENT_CAP)
    metric, ideal, orientation, q, records = CAPTURED[0]
    centers = []
    gaps = []
    from cftuv_envelope.reference.direction_binding import _density_decimal_envelope

    for ordinal, (use_x, _, lower, upper) in enumerate(records, start=1):
        env = _density_decimal_envelope(fan._slope(ideal[ordinal], use_x, metric), metric)
        center = (Fraction(env.lower) + Fraction(env.upper)) / 2
        gap = min(center - Fraction(lower.upper), Fraction(upper.lower) - center)
        centers.append(center)
        gaps.append(gap)
        print(f"  hidden ordinal {ordinal}: slope~{float(center):.12f} (irrational? {not fan._slope(ideal[ordinal], use_x, metric).is_Rational}) window gap {float(gap):.6e}")
    for k in (24, 25, 30, 40, 64, 119, 200, 400):
        divisor = 1 << k
        boxes = tuple(
            (c - g / divisor, c + g / divisor, records[i][0], records[i][1])
            for i, (c, g) in enumerate(zip(centers, gaps))
        )
        why, excess = reason(metric, ideal, orientation, q, boxes)
        print(f"  divisor 2^{k}: {why}" + ("" if excess is None else f"; worst step excess over {180.0/q:.0f}deg = {excess:.3e} deg"))


if __name__ == "__main__":
    main()
