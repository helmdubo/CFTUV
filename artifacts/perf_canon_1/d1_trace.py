"""walls.012, Fan Density 1: кто и сколько считает — FAN, факторизация, стадии."""

from __future__ import annotations

import sys
import time
from pathlib import Path

sys.path.insert(
    0,
    str(Path(__file__).resolve().parents[1] / "perf_prepare_diag"),
)

import env  # noqa: F401,E402

from cftuv_envelope import exact_sqrt_sum as ess  # noqa: E402
from cftuv_envelope.reference import adaptive_density_fan as fan  # noqa: E402
from cftuv_envelope.reference import compile as refcompile  # noqa: E402
from cftuv_envelope.wavefront import skeleton as skel  # noqa: E402

T0 = time.perf_counter()


def stamp() -> str:
    return f"[{time.perf_counter() - T0:8.2f}s]"


_fan_prepared = fan._certify_adaptive_density_fan_prepared


def fan_spy(metric, ideal, orientation, q, records, **kwargs):
    started = time.perf_counter()
    print(f"{stamp()} FAN start q={q} windows={len(records)}", flush=True)
    try:
        result = _fan_prepared(metric, ideal, orientation, q, records, **kwargs)
    except Exception as exc:  # noqa: BLE001
        print(
            f"{stamp()} FAN {type(exc).__name__} {str(exc)[:90]} "
            f"{time.perf_counter() - started:.2f}s",
            flush=True,
        )
        raise
    print(
        f"{stamp()} FAN ok D*={result.minimal_common_height} "
        f"{time.perf_counter() - started:.2f}s",
        flush=True,
    )
    return result


fan._certify_adaptive_density_fan_prepared = fan_spy

_legacy = fan._legacy_density_certificate


def legacy_spy(metric, ideal, ordinal, q, record):
    started = time.perf_counter()
    result = _legacy(metric, ideal, ordinal, q, record)
    elapsed = time.perf_counter() - started
    print(
        f"{stamp()} LEGACY ordinal={ordinal} q={q} -> "
        f"{'cert' if result is not None else 'None'} {elapsed:.2f}s",
        flush=True,
    )
    return result


fan._legacy_density_certificate = legacy_spy

_factorize = ess._factorize


def factorize_spy(value):
    started = time.perf_counter()
    result = _factorize(value)
    elapsed = time.perf_counter() - started
    if elapsed > 0.05:
        largest = max(result) if result else 1
        print(
            f"{stamp()} FACT {value.bit_length():4d} bits {elapsed:8.2f}s "
            f"maxprime={largest.bit_length()} bits",
            flush=True,
        )
    return result


ess._factorize = factorize_spy

_compile = refcompile.compile_reference_envelopes


def compile_spy(*args, **kwargs):
    started = time.perf_counter()
    print(f"{stamp()} COMPILE start", flush=True)
    result = _compile(*args, **kwargs)
    print(
        f"{stamp()} COMPILE done {time.perf_counter() - started:.2f}s",
        flush=True,
    )
    return result


refcompile.compile_reference_envelopes = compile_spy

_build_skeleton = skel.build_skeleton


def skeleton_spy(*args, **kwargs):
    started = time.perf_counter()
    print(f"{stamp()} SKELETON start", flush=True)
    result = _build_skeleton(*args, **kwargs)
    print(
        f"{stamp()} SKELETON done {time.perf_counter() - started:.2f}s",
        flush=True,
    )
    return result


skel.build_skeleton = skeleton_spy
import cftuv_envelope.wavefront.conveyor as conveyor  # noqa: E402

conveyor.build_skeleton = skeleton_spy

from counterfactual import PATH  # noqa: E402
from run_domain import (  # noqa: E402
    bundle_from_field_snapshot,
    snapshot_and_request,
    timed_queue,
)
from snapshot_bmesh import load_snapshot  # noqa: E402

DENSITY = int(sys.argv[1]) if len(sys.argv) > 1 else 1


def main():
    payload = load_snapshot(PATH)
    selected = frozenset(payload["raw"]["selected_edges"])
    _, _, bundle = bundle_from_field_snapshot(PATH)
    rows = snapshot_and_request(bundle, selected, density=DENSITY)
    patch_id, domain_id, snapshot, request = rows[0]
    print(f"{stamp()} host export done", flush=True)
    prepared, domain, total = timed_queue(patch_id, domain_id, snapshot, request)
    print(
        f"{stamp()} DONE {domain.preparation_outcome} / "
        f"{domain.coverage_outcome} / {domain.detail} total={total:.2f}s",
        flush=True,
    )


if __name__ == "__main__":
    main()
