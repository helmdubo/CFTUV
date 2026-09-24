"""Все ДВЕРИ канонизации: кадр ядра над каждым ПРОМАХОМ мемо, память холодная.

Отличие от `factorize_callers.py`: мемо сбрасывается перед каждым доменом, а
промах определяется теневым множеством уже виденных чисел, поэтому видно не
только ту дверь, которой повезло прийти первой в тёплом процессе, а ВСЕ, через
которые работа физически может пойти.
"""

from __future__ import annotations

import json
import sys
import traceback
from collections import Counter

import env  # noqa: F401

from cftuv_envelope import exact_sqrt_sum as ess


DOORS: dict[str, Counter[str]] = {}
BITS: dict[str, dict[str, int]] = {}
SEEN: dict[str, set[int]] = {}


def _kernel_frame():
    stack = traceback.extract_stack()
    chain = []
    for frame in reversed(stack):
        if frame.filename.endswith("exact_sqrt_sum.py"):
            chain.append(frame.name)
            continue
        if frame.filename.endswith("canon_doors.py"):
            continue
        if chain:
            short = frame.filename.split("cftuv_envelope/")[-1]
            if "cftuv_envelope" not in frame.filename:
                short = frame.filename.split("/")[-1]
            return f"{short}:{frame.name} <- " + " <- ".join(chain[::-1])
    return "?"


def _record(kind, n):
    key = _kernel_frame()
    DOORS.setdefault(kind, Counter())[key] += 1
    bits = BITS.setdefault(kind, {})
    bits[key] = max(bits.get(key, 0), int(n).bit_length())


def _spy_miss(kind, name):
    original = getattr(ess, name)
    SEEN[kind] = set()

    def traced(n, *args, **kwargs):
        if n not in SEEN[kind]:
            SEEN[kind].add(n)
            _record(kind, n)
        return original(n, *args, **kwargs)

    traced.cache_clear = getattr(original, "cache_clear", lambda: None)
    setattr(ess, name, traced)
    return original


def _spy_every(kind, name):
    original = getattr(ess, name)

    def traced(n, *args, **kwargs):
        _record(kind, n)
        return original(n, *args, **kwargs)

    setattr(ess, name, traced)
    return original


_spy_miss("squarefree_split_MISS", "squarefree_split")
_spy_miss("prime_support_MISS", "prime_support")
_spy_miss("factorization_pairs_MISS", "_factorization_pairs")
_spy_every("rho_factors", "_rho_factors")


def _reset():
    ess.reset_factorization_memory()
    for value in SEEN.values():
        value.clear()


def _report():
    print(json.dumps(
        {
            kind: [
                {"calls": count, "max_bits": BITS[kind][key], "where": key}
                for key, count in counter.most_common()
            ]
            for kind, counter in DOORS.items()
        },
        ensure_ascii=False,
        indent=2,
    ))


def main():
    from run_domain import bundle_from_field_snapshot, snapshot_and_request
    from snapshot_bmesh import load_snapshot
    from cftuv.envelope_queue_export import run_queue_domain

    name = sys.argv[1] if len(sys.argv) > 1 else "walls_012_snapshot.json"
    if name.startswith("building"):
        import big_scene

        _reset()
        sys.argv = ["big_scene", "3", sys.argv[2], "3000", name.split(":")[1]]
        big_scene.main()
        _report()
        return
    density = int(sys.argv[2]) if len(sys.argv) > 2 else 0
    path = env.SNAPSHOTS / name
    payload = load_snapshot(path)
    selected = frozenset(payload["raw"]["selected_edges"])
    _, _, bundle = bundle_from_field_snapshot(path)
    rows = snapshot_and_request(bundle, selected, density=density)
    for patch_id, domain_id, snapshot, request in rows:
        _reset()
        prepared, domain = run_queue_domain(
            patch_id, domain_id, snapshot, request, "0.45"
        )
        print(
            "patch", patch_id, domain.preparation_outcome,
            domain.coverage_outcome,
        )
    _report()


if __name__ == "__main__":
    main()
