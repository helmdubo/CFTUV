"""Сборка RECEIPT.json из сырых прогонов (каталог с ними — аргумент).

    python make_receipt.py <scratch_dir> <out RECEIPT.json>

Читает: baseline.json + baseline.time (sweep.py), seq_fp.json (тот же цикл в
процессе, с отпечатками), pool_N.json / pool2_N.json (по два прохода: холодный и
тёплый), pool3_N_rR.json (один проход), а также pickle.json и profile.json, если
они собраны. Равенство считается `compare.py` заново по каждому проходу.
"""

from __future__ import annotations

import copy
import json
import re
import statistics
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import compare  # noqa: E402

SCR = Path(sys.argv[1])
OUT = Path(sys.argv[2])


def sim_makespan(durations, workers, startup=0.0):
    """Жадное расписание «самый длинный первым» на `workers` без контенции."""

    load = [startup] * workers
    for seconds in sorted(durations, reverse=True):
        index = load.index(min(load))
        load[index] += seconds
    return max(load)


def main():
    base_raw = json.loads((SCR / "baseline.json").read_text(encoding="utf-8"))
    base = compare.load_rows(SCR / "baseline.json")[0]
    seq = compare.load_rows(SCR / "seq_fp.json")[0]
    seq_raw = json.loads((SCR / "seq_fp.json").read_text(encoding="utf-8"))
    durations = [row["seconds"] for row in base.values()]
    match = re.search(
        r"real\s+(\d+)m([\d.]+)s", (SCR / "baseline.time").read_text()
    )
    baseline_wall = int(match.group(1)) * 60 + float(match.group(2))
    sha = subprocess.run(
        ["git", "rev-parse", "HEAD"], capture_output=True, text=True, cwd=HERE
    ).stdout.strip()

    runs: dict[int, list] = {}
    for path in sorted(SCR.glob("pool*_*.json")):
        stem = path.stem
        if stem.startswith("pool_pickle"):
            continue
        n = int(stem.split("_")[1])
        rec = json.loads(path.read_text(encoding="utf-8"))
        passes = rec["passes"]
        runs.setdefault(n, []).append(
            (
                stem,
                passes[0]["wall_seconds"],
                passes[1]["wall_seconds"] if len(passes) > 1 else None,
                rec,
            )
        )

    mismatches = []
    compared = 0
    for n, items in runs.items():
        for stem, _, _, rec in items:
            for index, entry in enumerate(rec["passes"]):
                rows = {row["patch_id"]: row for row in entry["rows"]}
                d1 = compare.compare_to_sweep(base, rows)
                d2 = compare.compare_to_seq(seq, rows)
                compared += 1
                if d1 or d2:
                    mismatches.append(
                        {"run": f"{stem}#pass{index}", "sweep": d1[:5], "seq": d2[:5]}
                    )

    # Отрицательный контроль: сравнение обязано ЛОВИТЬ подмену.
    first_rows = runs[min(runs)][0][3]["passes"][0]["rows"]
    victim = next(r["patch_id"] for r in first_rows if "EXACT_WORK_SPENT" in r)
    rows = {r["patch_id"]: copy.deepcopy(r) for r in first_rows}
    rows[victim]["EXACT_WORK_GCD_OPERATIONS"] += 1
    caught_counter = bool(compare.compare_to_sweep(base, rows))
    rows = {r["patch_id"]: copy.deepcopy(r) for r in first_rows}
    rows[victim]["fp_geometry"] = "x"
    caught_fp = bool(compare.compare_to_seq(seq, rows))
    seq_vs_sweep = compare.compare_to_sweep(base, seq)

    startups = [
        row["worker_startup"]
        for items in runs.values()
        for _, _, _, rec in items
        for row in rec["passes"][0]["rows"]
        if "worker_startup" in row
    ]
    per_n = {}
    for n in sorted(runs):
        items = runs[n]
        cold = [wall for _, wall, _, _ in items]
        warm = [wall for _, _, wall, _ in items if wall is not None]
        rss_peak, rss_sum, init_seconds, ready, ratio6 = [], [], [], [], []
        for _, _, _, rec in items:
            t0 = rec["t_created"]
            per_pid = {}
            for row in rec["passes"][0]["rows"]:
                per_pid[row["pid"]] = max(per_pid.get(row["pid"], 0), row["peak_wset_mb"])
                if "worker_startup" in row:
                    info = row["worker_startup"]
                    init_seconds.append(info["init_seconds"])
                    ready.append(info["t_init_done"] - t0)
            rss_peak.append(max(per_pid.values()))
            rss_sum.append(sum(per_pid.values()))
            by_id = {r["patch_id"]: r["seconds"] for r in rec["passes"][0]["rows"]}
            ratio6.append(round(by_id[6] / base[6]["seconds"], 2))
        per_n[str(n)] = {
            "runs": len(items),
            "cold_wall_samples": cold,
            "warm_wall_samples": warm,
            "cold_wall_median": round(statistics.median(cold), 2),
            "cold_wall_min": min(cold),
            "warm_wall_median": round(statistics.median(warm), 2) if warm else None,
            "warm_wall_min": min(warm) if warm else None,
            "speedup_cold_median": round(baseline_wall / statistics.median(cold), 2),
            "speedup_cold_best": round(baseline_wall / min(cold), 2),
            "speedup_warm_best": round(baseline_wall / min(warm), 2) if warm else None,
            "pool_ready_after_creation_seconds_max": round(max(ready), 2),
            "worker_init_seconds_median": round(statistics.median(init_seconds), 2),
            "worker_init_seconds_max": round(max(init_seconds), 2),
            "peak_working_set_mb_per_worker_max": max(rss_peak),
            "peak_working_set_mb_sum_over_workers_max": round(max(rss_sum), 1),
            "patch6_seconds_over_baseline_ratio_samples": ratio6,
            "ideal_makespan_no_contention_with_0p5s_startup": round(
                sim_makespan(durations, n, startup=0.5), 2
            ),
        }

    slowest = sorted(base.items(), key=lambda kv: -kv[1]["seconds"])
    top10 = []
    for patch_id, row in slowest[:10]:
        full = seq.get(patch_id, {})
        counters = full.get("prepared_counters", {})
        top10.append(
            {
                "patch_id": patch_id,
                "seconds_sweep_py": row["seconds"],
                "seconds_in_process_loop": full.get("seconds"),
                "host_export_seconds": full.get("host_export_seconds"),
                "prepare_seconds": full.get("prepare_seconds"),
                "coverage_seconds": full.get("coverage_seconds"),
                "work_units_spent": row.get("EXACT_WORK_SPENT"),
                "outcome": row["outcome"],
                "domain_edges": counters.get("CONVEYOR_DOMAIN_EDGES"),
                "skeleton_nodes": counters.get("CONVEYOR_SKELETON_NODES"),
                "reflex_rational_vertex_fans": counters.get("CONVEYOR_RATIONAL_VERTEX_FANS"),
                "faces": counters.get("CONVEYOR_FACES"),
            }
        )

    receipt = {
        "commit": sha,
        "machine": {
            "cpu": "AMD Ryzen 9 5900X 12-Core",
            "logical_cores": seq_raw["machine_cores"],
            "physical_cores": 12,
            "python": "3.13.1 system python, multiprocessing spawn",
        },
        "route": "artifacts/building_full_sweep/sweep.py per-domain code, alpha 0.45, standard work cap",
        "baseline": {
            "sweep_py_wall_seconds_real": round(baseline_wall, 1),
            "sweep_py_sum_domain_seconds": round(sum(durations), 3),
            "sweep_py_max_domain_seconds": max(durations),
            "in_process_loop_wall_seconds": seq_raw["passes"][0]["wall_seconds"],
            "domains": len(base),
            "outcomes": {
                k: sum(1 for r in base.values() if r["outcome"] == k)
                for k in sorted({r["outcome"] for r in base.values()})
            },
            "speedup_upper_bound_by_slowest_domain": round(baseline_wall / max(durations), 2),
        },
        "equality": {
            "verdict": "MISMATCH" if (mismatches or seq_vs_sweep) else "IDENTICAL",
            "compared_against": (
                "sweep.py rows (outcome, six budget articles, EXACT_WORK_SPENT, "
                "leaked_unbudgeted, detail) and the in-process reference loop "
                "(answer fingerprints of regions/faces/segments/counters/meta and "
                "full prepared.counters)"
            ),
            "pool_passes_compared": compared,
            "domains_per_pass": len(base),
            "in_process_loop_vs_sweep_py_diffs": len(seq_vs_sweep),
            "mismatches": mismatches,
            "negative_control_counter_bump_caught": caught_counter,
            "negative_control_fingerprint_bump_caught": caught_fp,
        },
        "pool_by_workers": per_n,
        "top10_slowest_domains": top10,
        "bpy": {
            "note": "workers import only the conftest stub ModuleType('bpy') (no __file__); real bpy never imported",
            "workers_checked": len(startups),
            "all_workers_bpy_is_stub": all(s["bpy_is_stub"] for s in startups),
        },
        "amdahl": {
            "top5_domains_seconds_share": round(
                sum(sorted(durations, reverse=True)[:5]) / sum(durations), 3
            ),
            "top5_domains_seconds": sorted(durations, reverse=True)[:5],
            "note": "domains 6,1,7,11,15 carry ~76% of serial time; once workers >= 5 the wall is the slowest domain (patch6), so more workers do not help",
        },
    }
    bl_seq_path = SCR / "bl311_seq.json"
    if bl_seq_path.exists():
        bl_seq = json.loads(bl_seq_path.read_text(encoding="utf-8"))
        bl_seq_wall = bl_seq["passes"][0]["wall_seconds"]
        bl_runs = {}
        bl_diffs = []
        for path in sorted(SCR.glob("bl311_pool_*.json")):
            n = int(path.stem.split("_")[2])
            rec = json.loads(path.read_text(encoding="utf-8"))
            rows = {r["patch_id"]: r for r in rec["passes"][0]["rows"]}
            bl_diffs += compare.compare_to_sweep(base, rows)
            bl_diffs += compare.compare_to_seq(seq, rows)
            entry = bl_runs.setdefault(str(n), {"wall_samples": [], "patch6_seconds": [], "peak_working_set_mb_per_worker_max": 0})
            entry["wall_samples"].append(rec["passes"][0]["wall_seconds"])
            entry["patch6_seconds"].append(rows[6]["seconds"])
            entry["peak_working_set_mb_per_worker_max"] = max(
                entry["peak_working_set_mb_per_worker_max"],
                max(r["peak_wset_mb"] for r in rows.values()),
            )
        seq_rows = {r["patch_id"]: r for r in bl_seq["passes"][0]["rows"]}
        bl_diffs += compare.compare_to_sweep(base, seq_rows)
        bl_diffs += compare.compare_to_seq(seq, seq_rows)
        for entry in bl_runs.values():
            entry["wall_median"] = round(statistics.median(entry["wall_samples"]), 2)
            entry["speedup_vs_same_python_sequential_median"] = round(
                bl_seq_wall / statistics.median(entry["wall_samples"]), 2
            )
        receipt["blender_bundled_python_3_11"] = {
            "interpreter": "C:/Program Files/Blender Foundation/Blender 4.5/4.5/python/bin/python.exe (standalone, not inside Blender), python " + bl_seq["python"],
            "sequential_in_process_wall_seconds": bl_seq_wall,
            "sequential_slowest_domains_seconds_6_1_7_11_15": [seq_rows[k]["seconds"] for k in (6, 1, 7, 11, 15)],
            "slowdown_vs_python_3_13_sequential": round(bl_seq_wall / seq_raw["passes"][0]["wall_seconds"], 2),
            "pool": bl_runs,
            "equality_vs_3_13_baseline": "IDENTICAL" if not bl_diffs else "MISMATCH",
            "equality_diffs": bl_diffs[:5],
            "note": "same pool_sweep.py, psutil absent (ctypes working-set fallback); sympy/mpmath come from the user site-packages of Python311",
        }
    for name in ("pickle", "profile", "inputs_probe"):
        path = SCR / f"{name}.json"
        if path.exists():
            receipt[name] = json.loads(path.read_text(encoding="utf-8"))
    OUT.write_text(json.dumps(receipt, ensure_ascii=False, indent=1), encoding="utf-8")
    print(
        "written", OUT, OUT.stat().st_size, "bytes; equality:",
        receipt["equality"]["verdict"], "negctl:", caught_counter, caught_fp,
    )


main()
