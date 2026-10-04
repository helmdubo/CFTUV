"""Экспорты мешей сцены (слепки полевых мешей) под тремя символьными бэкендами: ответы равны, расхождений нет.

Меши: `walls.012`, `2.001` (стена, 35 выделенных рёбер), `walls.001` (U-маршрут) из
`artifacts/field_snapshots`. Каждый домен каждого слепка на каждой плотности считается тем же
маршрутом, что и кнопка (`snapshot_and_request` -> `run_queue_domain`), под `SYMPY`, затем `SHADOW`
(политика записи расхождений) и `NATIVE_EXACT`. Сверяется ПОЛНЫЙ отпечаток ответа домена
(`gate._answer_of`: исход, геометрия, счётчики, глубокие отпечатки внутренностей подготовки);
`building` в этот прогон не входит — у него ворота `numeric_repr/gate.py` и `materialize_sweep/sweep.py`.

    python artifacts/sympy_off_hot_path/scene_exports.py --densities 0,1,2,4 --out scene_exports.json
"""

from __future__ import annotations

import argparse
import collections
import json
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
for _entry in (
    ROOT / "artifacts" / "numeric_repr",
    ROOT / "artifacts" / "parallel_domains_spike",
    ROOT / "artifacts" / "perf_prepare_diag",
):
    if str(_entry) not in sys.path:
        sys.path.insert(0, str(_entry))

import env  # noqa: E402,F401  (пути и заглушки bpy)

from run_domain import (  # noqa: E402
    bundle_from_field_snapshot,
    snapshot_and_request,
    timed_queue,
)
from snapshot_bmesh import load_snapshot, selected_edge_ids  # noqa: E402

import gate  # noqa: E402

SNAPSHOTS = ("walls_012_snapshot", "wall_2_001_snapshot", "walls_001_u_route_snapshot")
MODES = ("SYMPY", "SHADOW", "NATIVE_EXACT")


def _run_domain(mode, patch_id, domain_id, snapshot, request, planar_types, backend):
    backend.reset_backend_counts()
    planar_types.TEXT_DIFFERENCES.clear()
    backend.set_backend_mode(backend.SymbolicBackendV1(mode))
    started = time.perf_counter()
    try:
        prepared, domain, _total = timed_queue(patch_id, domain_id, snapshot, request)
        answer = gate._answer_of(prepared, domain)
    except Exception as error:  # noqa: BLE001 - исход домена, а не авария прогона
        answer = {"outcome": f"EXCEPTION/{type(error).__name__}", "detail": str(error)[:300]}
    finally:
        backend.set_backend_mode(backend.SymbolicBackendV1.SYMPY)
    return {
        "answer": answer,
        "seconds": round(time.perf_counter() - started, 3),
        "counts": dict(backend.BACKEND_COUNTS),
        "disagreements": list(backend.DISAGREEMENTS),
        "text_differences": list(planar_types.TEXT_DIFFERENCES),
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--densities", default="0,1,2,4")
    parser.add_argument("--snapshots", default=",".join(SNAPSHOTS))
    parser.add_argument("--out", default="")
    args = parser.parse_args()

    from cftuv_envelope.reference import planar_types, symbolic_backend as backend

    backend.set_disagreement_policy(backend.DisagreementPolicyV1.RECORD)
    totals = {mode: collections.Counter() for mode in MODES}
    seconds = {mode: 0.0 for mode in MODES}
    rows = []
    mismatches = []
    disagreements = []
    text_differences = 0
    for name in args.snapshots.split(","):
        path = env.SNAPSHOTS / f"{name}.json"
        payload = load_snapshot(path)
        selected = frozenset(selected_edge_ids(payload))
        _payload, _bm, bundle = bundle_from_field_snapshot(path)
        for density in (int(value) for value in args.densities.split(",")):
            for patch_id, domain_id, snapshot, request in snapshot_and_request(
                bundle, selected, density=density
            ):
                results = {
                    mode: _run_domain(mode, patch_id, domain_id, snapshot, request, planar_types, backend)
                    for mode in MODES
                }
                for mode in MODES:
                    totals[mode].update(results[mode]["counts"])
                    seconds[mode] += results[mode]["seconds"]
                disagreements += results["SHADOW"]["disagreements"]
                text_differences += len(results["SHADOW"]["text_differences"])
                base = results["SYMPY"]["answer"]
                same = {mode: results[mode]["answer"] == base for mode in MODES}
                row = {
                    "snapshot": name,
                    "density": density,
                    "patch": patch_id,
                    "outcome": base.get("outcome"),
                    "answer_same": same,
                    "seconds": {mode: results[mode]["seconds"] for mode in MODES},
                }
                rows.append(row)
                if not all(same.values()):
                    mismatches.append(row)
                print(
                    f"{name} d{density} patch {patch_id}: {base.get('outcome')} "
                    f"same={same['SHADOW']}/{same['NATIVE_EXACT']} "
                    f"sympy {results['SYMPY']['seconds']:.2f}s native {results['NATIVE_EXACT']['seconds']:.2f}s",
                    flush=True,
                )
    checked = sum(v for k, v in totals["SHADOW"].items() if k.endswith(".shadow_checked"))
    summary = {
        "domains": len(rows),
        "answer_mismatches": len(mismatches),
        "shadow_checked_calls": checked,
        "disagreements": len(disagreements),
        "text_differences": text_differences,
        "shadow_counts": dict(sorted(totals["SHADOW"].items())),
        "native_counts": dict(sorted(totals["NATIVE_EXACT"].items())),
        "seconds_sum": {mode: round(value, 2) for mode, value in seconds.items()},
    }
    print(json.dumps(summary, indent=1))
    if args.out:
        Path(args.out).write_text(
            json.dumps({"summary": summary, "rows": rows, "mismatches": mismatches,
                        "disagreements": disagreements}, indent=1),
            encoding="utf-8",
        )
    return 1 if mismatches or disagreements else 0


if __name__ == "__main__":
    raise SystemExit(main())
