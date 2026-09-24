"""Один прогон walls.012 на одной Fan Density (аргумент), с профилем стадий."""

from __future__ import annotations

import json
import sys

import env  # noqa: F401
from run_domain import bundle_from_field_snapshot, snapshot_and_request, timed_queue, report
from snapshot_bmesh import load_snapshot


def main():
    raw = sys.argv[1]
    density = None if raw == "none" else int(raw)
    path = env.SNAPSHOTS / "walls_012_snapshot.json"
    payload = load_snapshot(path)
    selected = frozenset(payload["raw"]["selected_edges"])
    _, _, bundle = bundle_from_field_snapshot(path)
    rows = snapshot_and_request(bundle, selected, density=density)
    patch_id, domain_id, snapshot, request = rows[0]
    prepared, domain, total = timed_queue(patch_id, domain_id, snapshot, request)
    payload = report(prepared, domain, total)
    payload["density"] = raw
    print(json.dumps(payload, ensure_ascii=False))


if __name__ == "__main__":
    main()
