"""Сводит зонд pickle и профили трёх самых медленных доменов в pickle.json/profile.json.

    python assemble_extras.py <scratch_dir>
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

SCR = Path(sys.argv[1])
LARGEST_FIVE = (6, 1, 7, 11, 15)
SLOWEST_THREE = (6, 1, 7)


def load(path):
    return json.loads(Path(path).read_text(encoding="utf-8"))


def pickle_summary():
    record = load(SCR / "pool_pickle.json")
    light_check = {}
    rows = {row["patch_id"]: row for row in record["passes"][0]["rows"]}
    out = {}
    for patch_id in LARGEST_FIVE:
        probe = rows[patch_id]["pickle_probe"]
        ctx = load(SCR / "ctxprobe" / f"patch{patch_id}.probe.json")
        consume = load(SCR / "ctxprobe" / f"patch{patch_id}.consume.json")
        out[f"patch{patch_id}"] = {
            "ConveyorPreparationV1": {
                k: probe["prepared"][k]
                for k in ("ok", "error", "seconds")
                if k in probe["prepared"]
            },
            "EnvelopeQueueDomainV1_with_preparation": {
                k: probe["domain_with_preparation"][k]
                for k in ("ok", "error", "seconds")
                if k in probe["domain_with_preparation"]
            },
            "EnvelopeQueueDomainV1_light_preparation_None": {
                "ok": probe["domain_light"]["ok"],
                "bytes": probe["domain_light"].get("bytes"),
                "dumps_seconds": probe["domain_light"].get("dumps_seconds"),
                "loads_seconds": probe["domain_light"].get("loads_same_process_seconds"),
            },
            "preparation_fields_bytes_that_pickle": {
                k: v["bytes"]
                for k, v in probe["prepared_fields"].items()
                if v["ok"]
            },
            "preparation_fields_that_fail": [
                k for k, v in probe["prepared_fields"].items() if not v["ok"]
            ],
            "failing_object": ctx["offenders"],
            "memo_entries_dropped": ctx.get("density_exact_memo_entries"),
            "stripped_preparation_(metric._density_exact_memo emptied)": {
                "ok": ctx["stripped"]["ok"],
                "bytes": ctx["stripped"]["bytes"],
                "dumps_seconds": ctx["stripped"]["dumps_seconds"],
                "fresh_process_loads_seconds_first_includes_module_imports": consume[
                    "loads_seconds_first_in_fresh_process"
                ],
                "fresh_process_loads_seconds_second": consume["loads_seconds_second"],
                "slider_coverage_identical_alpha_0.45": consume["0.45"]["identical"],
                "slider_coverage_identical_alpha_0.30": consume["0.30"]["identical"],
                "coverage_seconds_consumer_vs_producer_0.45": [
                    consume["0.45"]["coverage_seconds_in_consumer"],
                    consume["0.45"]["coverage_seconds_in_producer"],
                ],
            },
        }
    return {
        "method": "worker = pool_sweep.py --pickle-dir (real ProcessPoolExecutor worker); "
        "stripped copy built with dataclasses.replace in pickle_probe.py (product code untouched); "
        "consumer = a fresh interpreter (different PYTHONHASHSEED and intern tables) that "
        "unpickles and runs conveyor_coverage + build_queue_domain at alpha 0.45 and 0.30; "
        "answers compared by fingerprint (geometry, meta, counters) against the producer",
        "domains_largest_five_by_seconds": out,
        "root_cause": "ConveyorPreparationV1.context.metric._density_exact_memo.intervals holds "
        "mpmath.ctx_iv.ivmpf (class created dynamically) -> PicklingError. Everything else in "
        "the preparation pickles (sympy expressions included). The memo is a per-transaction cache.",
    }


def profile_summary():
    seq = {
        row["patch_id"]: row
        for row in load(SCR / "seq_fp.json")["passes"][0]["rows"]
    }
    out = {}
    for patch_id in SLOWEST_THREE:
        record = load(SCR / f"profile_{patch_id}.json")
        row = seq[patch_id]
        out[f"patch{patch_id}"] = {
            "unprofiled_seconds": row["seconds"],
            "unprofiled_prepare_seconds": row["prepare_seconds"],
            "unprofiled_coverage_seconds": row["coverage_seconds"],
            "unprofiled_host_export_seconds": row["host_export_seconds"],
            "profiled_wall_seconds": record["profiled_wall_seconds"],
            "shape": record["shape"],
            "counters": record["counters"],
            "bucket_share_of_tottime": record["bucket_share"],
            "bucket_seconds_under_cProfile": record["bucket_seconds"],
            "top15_tottime": [
                {
                    "share": item["share"],
                    "ncalls": item["ncalls"],
                    "func": item["func"],
                    "bucket": item["bucket"],
                }
                for item in record["top15_tottime"]
            ],
        }
    return {
        "method": "cProfile of pool_sweep.compute_domain (host export + prepare + coverage), "
        "one fresh process per domain, three run concurrently; buckets by SELF time, "
        "builtins (math.gcd, isinstance, ...) attributed to the bucket of their caller",
        "caveat": "cProfile inflates call-heavy code ~3x (58.6 s profiled vs ~20 s unprofiled for "
        "patch6), so Fraction/dunder shares are upper bounds; the ordering is what matters",
        "domains": out,
    }


(SCR / "pickle.json").write_text(
    json.dumps(pickle_summary(), ensure_ascii=False, indent=1), encoding="utf-8"
)
(SCR / "profile.json").write_text(
    json.dumps(profile_summary(), ensure_ascii=False, indent=1), encoding="utf-8"
)
print("ok")
