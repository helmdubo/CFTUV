"""FIELD-STABLE-CANDIDATE: измерительная обвязка над artifacts/perf_prepare_diag.

Переиспользует run_domain.bundle_from_field_snapshot / snapshot_and_request /
timed_queue / report (без изменений) для ЛЮБОГО слепка в artifacts/field_snapshots,
с явными alpha и density. Ничего в проверяемом дереве не меняет.

Usage:
  python run_wall.py <path-to-perf_prepare_diag-dir> <snapshot-filename> <alpha> <density|none>
"""
from __future__ import annotations

import json
import sys
import time

HARNESS_DIR = sys.argv[1]
sys.path.insert(0, HARNESS_DIR)

import env  # noqa: E402  (сам вычисляет WT из своего __file__, т.е. из HARNESS_DIR)
from run_domain import bundle_from_field_snapshot, snapshot_and_request, timed_queue, report  # noqa: E402
from snapshot_bmesh import load_snapshot  # noqa: E402


def main():
    snapshot_name = sys.argv[2]
    alpha = float(sys.argv[3])
    density_raw = sys.argv[4]
    density = None if density_raw == "none" else int(density_raw)

    path = env.SNAPSHOTS / snapshot_name
    payload = load_snapshot(path)
    selected = frozenset(int(v) for v in payload["raw"]["selected_edges"])

    bundle_started = time.perf_counter()
    _, _, bundle = bundle_from_field_snapshot(path)
    bundle_seconds = time.perf_counter() - bundle_started

    rows = snapshot_and_request(bundle, selected, alpha=alpha, density=density)

    domains = []
    for patch_id, domain_id, snapshot, request in rows:
        prepared, domain, total = timed_queue(
            patch_id, domain_id, snapshot, request, alpha_text=str(alpha)
        )
        rep = report(prepared, domain, total)
        rep["patch_id"] = patch_id
        rep["domain_id"] = domain_id
        domains.append(rep)

    out = {
        "harness_dir": HARNESS_DIR,
        "snapshot": snapshot_name,
        "alpha": alpha,
        "density_raw": density_raw,
        "bundle_seconds": round(bundle_seconds, 4),
        "num_domains": len(rows),
        "domains": domains,
    }
    print(json.dumps(out, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
