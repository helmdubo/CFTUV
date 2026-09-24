"""РЕШАЮЩИЙ КОНТРФАКТ: цена стадий очереди против дефектов геометрии.

Варианты walls.012 (density передаётся аргументом):
  A_field          — слепок как есть (v0 в 1 см от плоскости стены,
                     колонна 5-10-6 уходит на 5 мм от вертикали);
  B_v0_flat        — v0 положен на плоскость стены (x := x остальных);
  C_column_straight— колонна 5-10-6 выпрямлена (y := y вершины 10);
  D_both           — оба дефекта устранены.
"""

from __future__ import annotations

import json
import sys
import time

import env  # noqa: F401
from run_domain import (
    bundle_from_field_snapshot,
    snapshot_and_request,
    timed_queue,
    report,
)
from snapshot_bmesh import load_snapshot

PATH = env.SNAPSHOTS / "walls_012_snapshot.json"


def variants():
    payload = load_snapshot(PATH)
    v = [tuple(item) for item in payload["raw"]["vertices"]]
    x_wall = v[1][0]
    y_col = v[10][1]
    return payload, {
        "A_field": None,
        "B_v0_flat": {0: (x_wall, v[0][1], v[0][2])},
        "C_column_straight": {
            5: (v[5][0], y_col, v[5][2]),
            6: (v[6][0], y_col, v[6][2]),
        },
        "D_both": {
            0: (x_wall, v[0][1], v[0][2]),
            5: (v[5][0], y_col, v[5][2]),
            6: (v[6][0], y_col, v[6][2]),
        },
    }


def main():
    density = int(sys.argv[1]) if len(sys.argv) > 1 else 0
    only = sys.argv[2] if len(sys.argv) > 2 else None
    payload, cases = variants()
    selected = frozenset(payload["raw"]["selected_edges"])
    results = {}
    for name, override in cases.items():
        if only and name != only:
            continue
        started = time.perf_counter()
        _, _, bundle = bundle_from_field_snapshot(PATH, vertex_override=override)
        host_seconds = time.perf_counter() - started
        rows = snapshot_and_request(bundle, selected, density=density)
        export_seconds = time.perf_counter() - started - host_seconds
        patch_id, domain_id, snapshot, request = rows[0]
        prepared, domain, total = timed_queue(patch_id, domain_id, snapshot, request)
        record = report(prepared, domain, total)
        record["variant"] = name
        record["density"] = density
        record["host_bundle_ms"] = round(host_seconds * 1000.0, 1)
        record["host_export_ms"] = round(export_seconds * 1000.0, 1)
        results[name] = record
        print(json.dumps(record, ensure_ascii=False))
    out = env.OUT / f"counterfactual_d{density}.json"
    previous = {}
    if out.exists():
        previous = json.loads(out.read_text(encoding="utf-8"))
    previous.update(results)
    out.write_text(
        json.dumps(previous, ensure_ascii=False, indent=2), encoding="utf-8"
    )


if __name__ == "__main__":
    main()
