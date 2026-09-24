"""Кто именно исчерпывает власть веера: перехват общего листа поиска D*."""

from __future__ import annotations

import json
import time
from fractions import Fraction

import env  # noqa: F401
from run_domain import bundle_from_field_snapshot, snapshot_and_request
from snapshot_bmesh import selected_edge_ids, load_snapshot

import cftuv_envelope as kernel
from cftuv_envelope.reference import adaptive_density_fan as fan

RECORDS = []


def bits(value) -> tuple[int, int]:
    frac = Fraction(value)
    return frac.numerator.bit_length(), frac.denominator.bit_length()


def _install():
    original = fan._certify_adaptive_density_fan_prepared

    def spy(metric, ideal, orientation, max_subturn_q, records, **kwargs):
        entry = {
            "orientation": str(orientation),
            "q": max_subturn_q,
            "ideal_count": len(ideal),
            "record_kinds": [type(item).__name__ for item in records],
        }
        started = time.perf_counter()
        try:
            result = original(
                metric, ideal, orientation, max_subturn_q, records, **kwargs
            )
        except fan.DensityRationalAuthorityExhausted as exc:
            entry["seconds"] = round(time.perf_counter() - started, 4)
            entry["outcome"] = f"EXHAUSTED: {exc}"
            RECORDS.append(entry)
            raise
        entry["seconds"] = round(time.perf_counter() - started, 4)
        entry["outcome"] = "OK"
        entry["minimal_common_height"] = result.minimal_common_height
        entry["termination_height_upper_bound"] = result.termination_height_upper_bound
        entry["window_kinds"] = [type(w).__name__ for w in result.ordinal_windows]
        entry["windows"] = [
            {
                "width_float": float(Fraction(*w.certified_termination_width)),
                "lower_bits": bits(Fraction(*w.termination_lower_slope)),
                "upper_bits": bits(Fraction(*w.termination_upper_slope)),
            }
            for w in result.ordinal_windows
        ]
        entry["fan"] = [str(v) for v in result.bound_primitive_integer_vectors]
        RECORDS.append(entry)
        return result

    fan._certify_adaptive_density_fan_prepared = spy


_install()


def run(label, vertex_override=None):
    RECORDS.clear()
    payload = load_snapshot(env.SNAPSHOTS / "walls_012_snapshot.json")
    _, _, bundle = bundle_from_field_snapshot(
        env.SNAPSHOTS / "walls_012_snapshot.json", vertex_override=vertex_override
    )
    rows = snapshot_and_request(bundle, selected_edge_ids(payload))
    patch_id, domain_id, snapshot, request = rows[0]
    started = time.perf_counter()
    result = kernel.compile_reference_envelopes(snapshot, request)
    elapsed = time.perf_counter() - started
    print(f"--- {label}: {result.outcome} in {elapsed:.3f}s")
    for item in RECORDS:
        print(json.dumps(item, ensure_ascii=False, default=str))
    return list(RECORDS)


if __name__ == "__main__":
    run("field walls.012")
