"""walls.012 на Fan Density 1 (значение UI по умолчанию): куда уходит время."""

from __future__ import annotations

import json
import time

import env  # noqa: F401
from cftuv_envelope.reference import adaptive_density_fan as fan

_original = fan._certify_adaptive_density_fan_prepared


def _spy(metric, ideal, orientation, q, records, **kwargs):
    started = time.perf_counter()
    print(f"FAN start q={q} ideal={len(ideal)}", flush=True)
    try:
        result = _original(metric, ideal, orientation, q, records, **kwargs)
    except Exception as exc:  # noqa: BLE001 — диагностика, не обработка
        print(
            f"FAN {type(exc).__name__} {str(exc)[:140]} "
            f"{time.perf_counter() - started:.2f}s",
            flush=True,
        )
        raise
    print(
        f"FAN ok D*={result.minimal_common_height} "
        f"term_ub={result.termination_height_upper_bound} "
        f"{time.perf_counter() - started:.2f}s",
        flush=True,
    )
    return result


fan._certify_adaptive_density_fan_prepared = _spy

from counterfactual import PATH  # noqa: E402
from run_domain import (  # noqa: E402
    bundle_from_field_snapshot,
    report,
    snapshot_and_request,
    timed_queue,
)
from snapshot_bmesh import load_snapshot  # noqa: E402


def main():
    payload = load_snapshot(PATH)
    selected = frozenset(payload["raw"]["selected_edges"])
    _, _, bundle = bundle_from_field_snapshot(PATH)
    started = time.perf_counter()
    rows = snapshot_and_request(bundle, selected, density=1)
    print(f"host export {time.perf_counter() - started:.2f}s", flush=True)
    patch_id, domain_id, snapshot, request = rows[0]
    prepared, domain, total = timed_queue(patch_id, domain_id, snapshot, request)
    print(json.dumps(report(prepared, domain, total), ensure_ascii=False), flush=True)


if __name__ == "__main__":
    main()
