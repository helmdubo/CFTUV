"""Скан: какие веера доходят до adaptive-ветки и чем кончают, по ВСЕМ доменам меша.

  PYTHONSAFEPATH=1 PYTHONPATH=kernel/src python artifacts/plan_not_compiled/scan_fans.py <root> <density> [--only p1,p2]

`<root>` — каталог выгрузки `export_domains.py ... ALL` (manifest.json +
patch_*/). На каждый домен перехватывается вход
`certify_density_bindings_and_adaptive_fallback` (по одному вызову на реверс-угол
с density-скрытыми рёбрами) и пишется строка:
  patch  outcome  fans=[ (steps, tight_steps, reasons, route) ... ]
route: LEGACY (B(w) закрыл веер), ADAPTIVE (ушло в V2-власть), REFUSED:<исход>.
`tight_steps` — число соседних пар идеального веера РОВНО на пределе Delta_max
(точный нуль остатка `_subturn_boundary`), `steps` — число пар.
"""
from __future__ import annotations

import json
import sys
import time
from pathlib import Path

from cftuv_envelope import AnalysisSnapshotCodecV1, DecalRequestCodecV1
from cftuv_envelope.reference import adaptive_density_fan as fan
from cftuv_envelope.reference.compile import compile_reference_envelopes

FANS = []


def _wrap():
    original = fan.certify_density_bindings_and_adaptive_fallback

    def wrapper(metric, ideal_unit_normals, orientation, q, *, binding_reasons, **kwargs):
        record = {"q": q, "reasons": [None if r is None else r.value for r in binding_reasons]}
        try:
            ideal = fan._covectors(metric, ideal_unit_normals)
            record["steps"] = len(ideal) - 1
            record["tight"] = sum(
                1 for i in range(len(ideal) - 1) if fan._subturn_boundary(metric, ideal[i], ideal[i + 1], q)
            )
        except Exception as exc:  # диагностика не должна менять ответ
            record["diag_error"] = type(exc).__name__
        FANS.append(record)
        try:
            certificates, authority = original(
                metric, ideal_unit_normals, orientation, q, binding_reasons=binding_reasons, **kwargs
            )
        except Exception as exc:
            record["route"] = "REFUSED:" + type(exc).__name__
            raise
        record["route"] = "LEGACY" if authority is None else "ADAPTIVE"
        record["certs_none"] = sum(1 for c in certificates if c is None)
        return certificates, authority

    fan.certify_density_bindings_and_adaptive_fallback = wrapper


def main():
    root = Path(sys.argv[1])
    density = int(sys.argv[2])
    only = None
    if "--only" in sys.argv:
        only = {int(x) for x in sys.argv[sys.argv.index("--only") + 1].split(",")}
    manifest = json.loads((root / "manifest.json").read_text(encoding="utf-8"))
    _wrap()
    totals = {}
    for patch in manifest["domains"]:
        if only is not None and int(patch) not in only:
            continue
        base = root / f"patch_{patch}"
        snapshot = AnalysisSnapshotCodecV1.loads((base / "analysis_snapshot.json").read_bytes())
        request = DecalRequestCodecV1.loads((base / f"decal_request_d{density}.json").read_bytes())
        FANS.clear()
        started = time.perf_counter()
        result = compile_reference_envelopes(snapshot, request)
        seconds = time.perf_counter() - started
        outcome = result.outcome.value
        tight = [f for f in FANS if f.get("tight")]
        key = (outcome, "tight" if tight else "no-tight", ",".join(sorted({f.get("route", "?") for f in FANS})) or "-")
        totals.setdefault(key, []).append(int(patch))
        summary = [(f["steps"], f.get("tight"), f["route"] if "route" in f else "?", f["reasons"]) for f in FANS if f.get("tight") or f.get("route", "") != "LEGACY"]
        print(f"p{patch} {outcome} {seconds:.2f}s fans={len(FANS)} {summary}")
    print("TOTALS")
    for key, patches in sorted(totals.items(), key=lambda kv: -len(kv[1])):
        print(" ", key, len(patches), patches[:40])


if __name__ == "__main__":
    main()
