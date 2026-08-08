"""Кто ДОСТИГАЕТ работы канонизации: кадр ядра над каждым промахом мемо.

Разведка карточки EXACT-WORK-BUDGET: чтобы протянуть бюджет по ЯВНОМУ
параметру, нужно знать не «кто вызывает `_factorize`», а КАКОЙ КАДР ЯДРА
стоит над каждой РАБОТОЙ. Мемо-попадания не считаются: они работы не делают
и бюджету не подлежат.
"""

from __future__ import annotations

import json
import sys
import time
import traceback
from collections import Counter

import env  # noqa: F401

from cftuv_envelope import exact_sqrt_sum as ess


CALLERS: dict[str, Counter[str]] = {}
BITS: dict[str, dict[str, int]] = {}
SECONDS: dict[str, dict[str, float]] = {}


def _kernel_frame():
    """Первый кадр ВНЕ exact_sqrt_sum.py, идя от вершины стека вниз."""

    stack = traceback.extract_stack()
    chain = []
    for frame in reversed(stack):
        if frame.filename.endswith("exact_sqrt_sum.py"):
            chain.append(frame.name)
            continue
        if chain:
            short = frame.filename.split("cftuv_envelope/")[-1]
            if "cftuv_envelope" not in frame.filename:
                short = frame.filename.split("/")[-1]
            return f"{short}:{frame.name} <- " + " <- ".join(chain[::-1])
    return "?"


def _wrap(kind, name, miss_only=False):
    original = getattr(ess, name)
    CALLERS[kind] = Counter()
    BITS[kind] = {}
    SECONDS[kind] = {}

    def traced(n, *args, **kwargs):
        if miss_only and n in _memo_of(name):
            return original(n, *args, **kwargs)
        key = _kernel_frame()
        started = time.perf_counter()
        try:
            return original(n, *args, **kwargs)
        finally:
            elapsed = time.perf_counter() - started
            CALLERS[kind][key] += 1
            BITS[kind][key] = max(BITS[kind].get(key, 0), int(n).bit_length())
            SECONDS[kind][key] = SECONDS[kind].get(key, 0.0) + elapsed

    setattr(ess, name, traced)


def _memo_of(name):
    cache = getattr(ess, name)
    try:
        return {}
    except Exception:  # pragma: no cover - разведка
        return {}


_wrap("rho_factors", "_rho_factors")
_wrap("pollard_rho", "_pollard_rho")


def main():
    which = sys.argv[1] if len(sys.argv) > 1 else "walls012"
    density = int(sys.argv[2]) if len(sys.argv) > 2 else 0
    if which in ("walls012", "wall2001"):
        from run_domain import (
            bundle_from_field_snapshot,
            snapshot_and_request,
            timed_queue,
        )
        from snapshot_bmesh import load_snapshot

        name = (
            "walls_012_snapshot.json"
            if which == "walls012"
            else "wall_2_001_snapshot.json"
        )
        path = env.SNAPSHOTS / name
        payload = load_snapshot(path)
        selected = frozenset(payload["raw"]["selected_edges"])
        _, _, bundle = bundle_from_field_snapshot(path)
        rows = snapshot_and_request(bundle, selected, density=density)
        for patch_id, domain_id, snapshot, request in rows:
            prepared, domain, total = timed_queue(
                patch_id, domain_id, snapshot, request
            )
            print(
                "patch", patch_id, "outcome:",
                domain.preparation_outcome, domain.coverage_outcome,
                "total_ms", round(total * 1000, 1),
            )
    else:
        import big_scene

        sys.argv = ["big_scene", "3", str(density), "3000", which]
        big_scene.main()

    print(json.dumps(
        {
            kind: [
                {
                    "calls": count,
                    "max_radicand_bits": BITS[kind][key],
                    "seconds": round(SECONDS[kind][key], 3),
                    "where": key,
                }
                for key, count in CALLERS[kind].most_common()
            ]
            for kind in CALLERS
        },
        ensure_ascii=False,
        indent=2,
    ))


if __name__ == "__main__":
    main()
