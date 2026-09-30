"""Вход воркера для схемы «хост собирает вход, пул считает»: (snapshot, request).

    PYTHONPATH=. python3 ../parallel_domains_spike/inputs_probe.py <out.json>

В Blender экспорт (`build_envelope_analysis_snapshot`, `build_envelope_decal_request`)
идёт в главном процессе — bundle живёт там. Воркеру уезжает пара
`(AnalysisSnapshotV1, DecalRequestV1)`. Мерим: сколько хост тратит на экспорт (это
последовательная часть закона Амдала), размер и время pickle пары, и что ответ
домена, посчитанный по РАСПИКЛЕННОЙ паре, совпадает отпечатком.
"""

from __future__ import annotations

import json
import pickle
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import pool_sweep  # noqa: E402

pool_sweep.init_worker(quiet=True)
ctx = pool_sweep._CTX
total_export = 0.0
rows = {}
refused = 0
for patch_id in sorted(ctx["patch_ids"]):
    domain_id = ctx["typed_value"]("patch-domain", ctx["revision"], patch_id)
    started = time.perf_counter()
    try:
        snapshot = ctx["build_snapshot"](
            ctx["bundle"], included_patch_ids=frozenset({patch_id})
        )
        request = ctx["build_request"](
            snapshot, frozenset(ctx["by_domain"][domain_id]), pool_sweep.ALPHA_VALUE,
            decal_request_id_value=ctx["request_id"], density=0,
        )
    except ctx["EnvelopeHostAdapterError"]:
        refused += 1
        total_export += time.perf_counter() - started
        continue
    export_seconds = time.perf_counter() - started
    total_export += export_seconds
    started = time.perf_counter()
    blob = pickle.dumps((snapshot, request), protocol=pickle.HIGHEST_PROTOCOL)
    dumps_seconds = time.perf_counter() - started
    started = time.perf_counter()
    pickle.loads(blob)
    loads_seconds = time.perf_counter() - started
    rows[patch_id] = {
        "export_seconds": round(export_seconds, 4),
        "pickle_bytes": len(blob),
        "dumps_seconds": round(dumps_seconds, 4),
        "loads_seconds": round(loads_seconds, 4),
    }

# Ответ по распикленному входу == ответ по исходному (один небольшой домен).
check = {}
for patch_id in (10, 100):
    domain_id = ctx["typed_value"]("patch-domain", ctx["revision"], patch_id)
    snapshot = ctx["build_snapshot"](
        ctx["bundle"], included_patch_ids=frozenset({patch_id})
    )
    request = ctx["build_request"](
        snapshot, frozenset(ctx["by_domain"][domain_id]), pool_sweep.ALPHA_VALUE,
        decal_request_id_value=ctx["request_id"], density=0,
    )
    snapshot2, request2 = pickle.loads(
        pickle.dumps((snapshot, request), protocol=pickle.HIGHEST_PROTOCOL)
    )
    answers = []
    for pair in ((snapshot, request), (snapshot2, request2)):
        ctx["canon"].reset_factorization_memory()
        _, domain = ctx["run_queue_domain"](
            patch_id, domain_id, pair[0], pair[1], pool_sweep.ALPHA_TEXT
        )
        answers.append(pool_sweep._fingerprints(domain))
    check[patch_id] = {"fingerprints_equal": answers[0] == answers[1]}

biggest = sorted(rows.items(), key=lambda kv: -kv[1]["pickle_bytes"])[:5]
summary = {
    "domains_exported": len(rows),
    "host_admission_refused": refused,
    "total_host_export_seconds_all_domains": round(total_export, 2),
    "pickle_bytes_max": max(v["pickle_bytes"] for v in rows.values()),
    "pickle_bytes_median": sorted(v["pickle_bytes"] for v in rows.values())[len(rows) // 2],
    "pickle_dumps_seconds_total": round(sum(v["dumps_seconds"] for v in rows.values()), 3),
    "pickle_loads_seconds_total": round(sum(v["loads_seconds"] for v in rows.values()), 3),
    "largest_five": {f"patch{k}": v for k, v in biggest},
    "roundtrip_answer_check": check,
}
Path(sys.argv[1]).write_text(json.dumps(summary, ensure_ascii=False, indent=1), encoding="utf-8")
print(json.dumps({k: v for k, v in summary.items() if k != "largest_five"}, ensure_ascii=False))
