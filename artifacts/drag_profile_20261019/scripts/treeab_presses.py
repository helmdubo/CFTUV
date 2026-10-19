"""Per-press seconds of blender_tree_ab worker reports (real cold/tick/incr/step presses): run_production wall + arrays, medians per kind, plus the plan of incr presses.

    python treeab_presses.py <runs dir> [--out summary.json]
"""
import json
import statistics
import sys
from pathlib import Path


def main():
    runs = Path(sys.argv[1])
    out = {}
    for path in sorted(runs.glob("*_w12_*_r1_a1.json")):
        report = json.loads(path.read_text(encoding="utf-8"))
        rows = []
        for label, press in report["presses"].items():
            rows.append({"label": label, "kind": press.get("kind") or ("cold" if label == "cold" else label.rstrip("0123456789")), "run": press["run_seconds"], "arrays": press["arrays_seconds"],
                         "exact": press["exact_seconds"], "domains": press["domains"], "materialized": press["materialized"], "plan": press.get("plan", {}).get("mode") if press.get("plan") else None,
                         "served": press.get("plan", {}).get("served") if press.get("plan") else None,
                         "stage_seconds": {k: v for k, v in press.get("stage_seconds", {}).items()},
                         "fast_hits": press.get("profile_counters", {}).get("PRODUCTION_INTERVAL_FAST_HITS"),
                         "result_cache_hits": press.get("profile_counters", {}).get("PRODUCTION_RESULT_CACHE_HITS"),
                         "alpha_intervals": {k: v for k, v in press.get("profile_counters", {}).items() if k.startswith("PRODUCTION_ALPHA_INTERVALS_")}})
        kinds = {}
        for kind in dict.fromkeys(r["kind"] for r in rows):
            sel = [r for r in rows if r["kind"] == kind]
            kinds[kind] = {"n": len(sel), "run_median": statistics.median(r["run"] for r in sel), "arrays_median": statistics.median(r["arrays"] for r in sel),
                           "exact_median": statistics.median(r["exact"] for r in sel), "run_all": [round(r["run"], 4) for r in sel], "plans": [(r["plan"], r["served"]) for r in sel],
                           "fast_hits": [r["fast_hits"] for r in sel], "alpha_intervals": sel[-1]["alpha_intervals"]}
        out[path.stem] = {"native": report.get("native_build_id"), "pool_prewarm_seconds": report.get("pool_prewarm_seconds"), "kinds": kinds, "rows": rows}
        print(path.stem, {k: (v["n"], round(v["run_median"], 4), round(v["arrays_median"], 4), round(v["exact_median"], 4)) for k, v in kinds.items()})
    if len(sys.argv) > 3 and sys.argv[2] == "--out":
        Path(sys.argv[3]).write_text(json.dumps(out, indent=1), encoding="utf-8")


if __name__ == "__main__":
    main()
