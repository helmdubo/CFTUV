"""Per-refresh split of a dragprof_runner report: parent laps, pool tasks attributed by time window, stage sums (worker-seconds), critical path.

    python analyze.py <report.json> [--out summary.json]
"""
import argparse
import json
import statistics
from collections import defaultdict
from pathlib import Path

# top-level worker stages (non-overlapping inside one task)
STAGES = {
    "EXPORT": ["EXPORT.task_inputs"],
    "PREPARE": ["PREPARE.total"],
    "PRODUCE.coverage": ["PRODUCE.coverage"],
    "PRODUCE.materialize": ["PRODUCE.materialize"],
    "PRODUCE.pack": ["PACK.deferred_result", "PACK.build_domain_view"],
    "REPLY": ["W.pack_prepared_reply", "W.reply_frame"],
}
SUB = ["prep.lap.PLAN_COMPILE", "prep.lap.DOMAIN_BUILD", "prep.lap.ARRIVAL_LAWS", "prep.lap.SKELETON", "prep.lap.BRIDGE", "prep.lap.FACES", "prep.lap.CHART_LATTICE",
       "cov.lap.EFFECTIVE_ALPHA", "cov.lap.COVERAGE_CLIP", "EA.resolve_component_alphas@PRODUCE", "EA.instance_ids_by_spec", "native.covered_at", "native.coverage_compute", "native.clip_compute",
       "native.w1_exact_mesh", "native.w1_canonical_triangles",
       "mat.lap.ADMIT", "mat.lap.STATIONS", "mat.lap.COALESCE", "mat.lap.CHORD_STATIONS", "mat.lap.FRAMES", "mat.lap.STATIONS_UV", "mat.lap.TESSELLATE", "mat.lap.SILHOUETTE", "mat.lap.ASSEMBLE",
       "mat.lap.VALIDATE", "mat.lap.AUDIT", "mat.lap.DIGEST", "mat.lap.CERTIFY", "mat.lap.PARAMETRIC", "S6A.certificate_of", "PRODUCE.step_domain", "W.task_unpickle"]
PARENT = ["begin.source_digest", "scan", "pool.order_by_cost", "dispatch", "retry", "P.run_production", "tail", "tail.sample", "tail.model", "tail.deviation", "tail.compose", "tail.origins",
          "plan", "plan.rebase", "arrays", "arrays.weld", "arrays.cross_seams", "arrays.digest", "arrays.off_plane", "arrays.seam_report", "arrays.half_edge", "P.apply", "apply.lean_frame",
          "apply.rewrite", "apply.mesh_digest"]


def load_tasks(directory: Path):
    tasks = []
    for path in directory.glob("*.jsonl"):
        for line in path.read_text(encoding="utf-8").splitlines():
            if line.strip():
                tasks.append(json.loads(line))
    return tasks


def med(values):
    values = list(values)
    return round(statistics.median(values), 5) if values else 0.0


def analyze(report_path: Path) -> dict:
    report = json.loads(report_path.read_text(encoding="utf-8"))
    tasks = load_tasks(Path(report["worker_dump_dir"])) if report.get("worker_dump_dir") else []
    per_step = []
    for step in report["steps"]:
        lo, hi = step["started_at"], step["finished_at"]
        mine = [t for t in tasks if lo <= t["t0"] <= hi]
        busy = defaultdict(float)
        stages = defaultdict(float)
        for t in mine:
            busy[t["pid"]] += t["t1"] - t["t0"]
            for stage, names in STAGES.items():
                stages[stage] += sum(t["laps"].get(n, 0.0) for n in names)
            for n in SUB:
                stages["sub." + n] += t["laps"].get(n, 0.0)
        serve = sum(t["t1"] - t["t0"] for t in mine)
        row = {
            "kind": step["kind"], "rep": step["rep"], "alpha": step["alpha"], "wall": step["wall"], "thread": step["thread"], "begin_main": step["begin_main"], "apply_main": step["apply_main"],
            "host_depsgraph": step.get("host_depsgraph"), "run_production": step["run_production"], "domains": step["domains"],
            "plan": step.get("plan"), "tasks": len(mine), "worker_serve_sum": serve, "worker_busiest": max(busy.values(), default=0.0),
            "task_max": max((t["t1"] - t["t0"] for t in mine), default=0.0), "pool_span": (max(t["t1"] for t in mine) - min(t["t0"] for t in mine)) if mine else 0.0,
            "tasks_with_prepare": sum(1 for t in mine if t["laps"].get("PREPARE.total", 0.0) > 0), "tasks_with_export": sum(1 for t in mine if t["laps"].get("EXPORT.task_inputs", 0.0) > 0),
            "stages_worker_s": {k: round(v, 5) for k, v in stages.items()},
            "parent": {n: step["laps"].get(n, [0, 0.0])[1] for n in PARENT},
        }
        row["worker_other"] = serve - sum(stages[s] for s in STAGES)
        per_step.append(row)
    kinds = sorted({row["kind"] for row in per_step}, key=[r["kind"] for r in per_step].index)
    summary = {}
    for kind in kinds:
        rows = [r for r in per_step if r["kind"] == kind]
        entry = {k: med(r[k] for r in rows) for k in ("wall", "thread", "begin_main", "apply_main", "host_depsgraph", "run_production", "tasks", "worker_serve_sum", "worker_busiest", "task_max", "pool_span", "tasks_with_prepare", "tasks_with_export", "worker_other")}
        entry["n"] = len(rows)
        entry["stages_worker_s"] = {k: med(r["stages_worker_s"].get(k, 0.0) for r in rows) for k in sorted({k for r in rows for k in r["stages_worker_s"]})}
        entry["parent"] = {k: med(r["parent"][k] for r in rows) for k in PARENT}
        entry["plans"] = [None if r["plan"] is None else {"served": r["plan"]["served"], "held": r["plan"]["held"], "composite": r["plan"]["composite"], "fallback": r["plan"]["fallback"], "reason": r["plan"]["reason"]} for r in rows]
        summary[kind] = entry
    return {"report": str(report_path), "cold_press_s": report.get("cold_press_s"), "decal": report.get("decal_cold"), "summary": summary, "per_step": per_step}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("report")
    parser.add_argument("--out")
    args = parser.parse_args()
    result = analyze(Path(args.report))
    text = json.dumps(result, indent=1)
    if args.out:
        Path(args.out).write_text(text, encoding="utf-8")
    for kind, entry in result["summary"].items():
        print(kind, {k: entry[k] for k in ("n", "wall", "thread", "begin_main", "apply_main", "host_depsgraph", "run_production", "tasks", "worker_serve_sum", "worker_busiest", "task_max", "pool_span", "tasks_with_prepare", "tasks_with_export")})
        print("   worker stages:", {k: v for k, v in entry["stages_worker_s"].items() if v >= 0.001})
        print("   parent:", {k: v for k, v in entry["parent"].items() if v >= 0.0005})


if __name__ == "__main__":
    main()
