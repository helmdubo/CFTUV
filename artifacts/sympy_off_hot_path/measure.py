"""Замер скорости символьных бэкендов на тяжёлых доменах `building`: холодная подготовка и тёплая alpha.

Каждое измерение — ОТДЕЛЬНЫЙ процесс (кэши `sympy`, `lru_cache` и память разложений не переходят из
режима в режим), режимы чередуются внутри каждого повтора, чтобы фоновая нагрузка машины делилась
между ними поровну. Процесс прогревается малым доменом (как воркер пула, который уже сделал
первую задачу), затем считает целевой домен:

    cold  — host export + `run_queue_domain` (подготовка + первое покрытие, alpha 0.45);
    warm  — `cover_prepared` на готовой подготовке при другой alpha (0.30, затем 0.45), лучший из двух.

Запуск:

    python artifacts/sympy_off_hot_path/measure.py --patches 6,7,1 --density 2 --reps 3 \
        --modes SYMPY,NATIVE_EXACT --out measure.json

Режим выбирает переменная `CFTUV_SYMBOLIC_BACKEND` (читает её только харнесс, см. `gate.py`).
"""

from __future__ import annotations

import argparse
import json
import os
import statistics
import subprocess
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

WARMUP_PATCH = 100
ALPHA_COLD = "0.45"
ALPHA_WARM = ("0.30", "0.45")


def child(patch_id: int, density: int) -> dict:
    import gate  # noqa: E402
    import pool_sweep  # noqa: E402

    gate.init_worker()
    gate.compute_row(WARMUP_PATCH, density)
    ctx = pool_sweep._CTX
    domain_id = ctx["typed_value"]("patch-domain", ctx["revision"], patch_id)
    ctx["canon"].reset_factorization_memory()
    ctx["canon"].reset_unbudgeted_work()
    started = time.perf_counter()
    snapshot = ctx["build_snapshot"](
        ctx["bundle"], included_patch_ids=frozenset({patch_id})
    )
    request = ctx["build_request"](
        snapshot,
        frozenset(ctx["by_domain"][domain_id]),
        pool_sweep.ALPHA_VALUE,
        decal_request_id_value=ctx["request_id"],
        density=density,
    )
    host = time.perf_counter() - started
    prepared, domain = ctx["run_queue_domain"](
        patch_id, domain_id, snapshot, request, ALPHA_COLD
    )
    cold = time.perf_counter() - started
    from cftuv.envelope_queue_export import cover_prepared

    warm = []
    for alpha in ALPHA_WARM:
        tick = time.perf_counter()
        cover_prepared(patch_id, domain_id, prepared, alpha)
        warm.append(time.perf_counter() - tick)
    return {
        "patch": patch_id,
        "outcome": f"{domain.preparation_outcome}/{domain.coverage_outcome}",
        "cold_seconds": round(cold, 3),
        "host_seconds": round(host, 3),
        "prepare_seconds": round(domain.prepare_seconds, 3),
        "first_coverage_seconds": round(domain.coverage_seconds, 3),
        "warm_seconds": [round(value, 4) for value in warm],
    }


def run(args) -> dict:
    modes = args.modes.split(",")
    patches = [int(value) for value in args.patches.split(",")]
    samples: dict[str, dict[int, list[dict]]] = {m: {p: [] for p in patches} for m in modes}
    for repetition in range(args.reps):
        for patch_id in patches:
            for mode in modes:
                env = dict(os.environ, CFTUV_SYMBOLIC_BACKEND="" if mode == "SYMPY" else mode)
                done = subprocess.run(
                    [sys.executable, str(Path(__file__).resolve()), "--child",
                     str(patch_id), str(args.density)],
                    env=env, capture_output=True, text=True, timeout=1500,
                )
                if done.returncode != 0:
                    raise SystemExit(done.stderr[-2000:])
                row = json.loads(done.stdout.strip().splitlines()[-1])
                samples[mode][patch_id].append(row)
                print(f"[rep {repetition}] {mode:13s} patch {patch_id}: cold {row['cold_seconds']:.2f}s "
                      f"warm {min(row['warm_seconds']):.3f}s ({row['outcome']})", flush=True)
    summary = {}
    for mode in modes:
        summary[mode] = {}
        for patch_id in patches:
            rows = samples[mode][patch_id]
            cold = [r["cold_seconds"] for r in rows]
            warm = [min(r["warm_seconds"]) for r in rows]
            summary[mode][str(patch_id)] = {
                "cold_median": round(statistics.median(cold), 3),
                "cold_min": round(min(cold), 3),
                "warm_median": round(statistics.median(warm), 4),
                "warm_min": round(min(warm), 4),
                "prepare_median": round(statistics.median(r["prepare_seconds"] for r in rows), 3),
                "first_coverage_median": round(statistics.median(r["first_coverage_seconds"] for r in rows), 3),
                "outcomes": sorted({r["outcome"] for r in rows}),
            }
    result = {"density": args.density, "reps": args.reps, "summary": summary, "samples": {
        m: {str(p): v for p, v in d.items()} for m, d in samples.items()}}
    if {"SYMPY", "NATIVE_EXACT"} <= set(modes):
        result["speedup"] = {
            str(p): {
                "cold_median": round(summary["SYMPY"][str(p)]["cold_median"] / summary["NATIVE_EXACT"][str(p)]["cold_median"], 3),
                "first_coverage_median": round(summary["SYMPY"][str(p)]["first_coverage_median"] / max(summary["NATIVE_EXACT"][str(p)]["first_coverage_median"], 1e-9), 3),
                "warm_median": round(summary["SYMPY"][str(p)]["warm_median"] / max(summary["NATIVE_EXACT"][str(p)]["warm_median"], 1e-9), 3),
            }
            for p in patches
        }
    return result


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--child", nargs=2, metavar=("PATCH", "DENSITY"))
    parser.add_argument("--patches", default="6,7,1")
    parser.add_argument("--density", type=int, default=2)
    parser.add_argument("--reps", type=int, default=3)
    parser.add_argument("--modes", default="SYMPY,NATIVE_EXACT")
    parser.add_argument("--out", default="")
    args = parser.parse_args()
    if args.child:
        print(json.dumps(child(int(args.child[0]), int(args.child[1]))))
        return 0
    result = run(args)
    print(json.dumps(result["summary"], indent=1))
    if "speedup" in result:
        print("speedup (SYMPY / NATIVE_EXACT):", json.dumps(result["speedup"]))
    if args.out:
        Path(args.out).write_text(json.dumps(result, indent=1), encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
