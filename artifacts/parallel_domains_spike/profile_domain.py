"""cProfile одного домена в процессе + разбиение времени по корзинам.

    PYTHONPATH=. python3 ../parallel_domains_spike/profile_domain.py <patch_id> <out.json> [pstats]

Маршрут — `pool_sweep.compute_domain` (тот же, что в sweep.py), профилируется
целиком: host export + подготовка + покрытие. Корзины считаются по СОБСТВЕННОМУ
времени (tottime). Встроенные функции (`math.gcd`, `pow`, `abs`, ...) профилировщик
видит без файла, поэтому их время относится к корзине ВЫЗЫВАЮЩЕГО (по таблице
вызывающих): gcd из `Fraction.__add__` — арифметика дробей, gcd из Поллард-rho —
теория чисел.
"""

from __future__ import annotations

import cProfile
import json
import pstats
import re
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import pool_sweep  # noqa: E402

NUMBER_THEORY = re.compile(
    r"is_prime|pollard|rho|register_prime|strip_known|factor|coprime|"
    r"squarefree|prime|_split_by_prime|modular",
    re.IGNORECASE,
)
SKELETON_FILES = re.compile(
    r"skeleton|motorcycle|superlevel|event|symbolic|poststate", re.IGNORECASE
)


def bucket_of(func) -> str | None:
    """Корзина функции; `None` — встроенная (решается по вызывающим)."""

    filename, _, name = func
    norm = filename.replace("\\", "/")
    if filename == "~":
        return None
    if "sympy" in norm:
        return "sympy"
    if norm.endswith("/fractions.py"):
        return "Fraction/gcd arithmetic"
    base = norm.rsplit("/", 1)[-1]
    if base == "exact_sqrt_sum.py":
        if NUMBER_THEORY.search(name):
            return "number theory"
        return "exact_sqrt_sum"
    if base == "sqrt_sum.py":
        return "exact_sqrt_sum"
    if "cftuv_envelope/wavefront/" in norm and SKELETON_FILES.search(base):
        return "skeleton march"
    return "other"


def classify(stats: pstats.Stats):
    totals: dict[str, float] = {}
    builtin_split: dict[str, dict[str, float]] = {}
    for func, (cc, nc, tt, ct, callers) in stats.stats.items():
        bucket = bucket_of(func)
        if bucket is not None:
            totals[bucket] = totals.get(bucket, 0.0) + tt
            continue
        # встроенная: делим её tottime по вызывающим пропорционально tt вызова
        weight = sum(entry[2] for entry in callers.values())
        for caller, entry in callers.items():
            target = bucket_of(caller) or "other"
            share = entry[2] if weight else 0.0
            totals[target] = totals.get(target, 0.0) + share
            builtin_split.setdefault(func[2], {})
            builtin_split[func[2]][target] = (
                builtin_split[func[2]].get(target, 0.0) + share
            )
        if not callers:
            totals["other"] = totals.get("other", 0.0) + tt
    return totals, builtin_split


def main():
    patch_id = int(sys.argv[1])
    out = sys.argv[2]
    pstats_path = sys.argv[3] if len(sys.argv) > 3 else None
    pool_sweep.init_worker(quiet=True)
    ctx = pool_sweep._CTX
    node = ctx["bundle"].patch_graph.nodes[patch_id]
    shape = {
        "faces": len(node.face_indices),
        "boundary_loops": len(node.boundary_loops),
        "boundary_edges": sum(len(l.edge_indices) for l in node.boundary_loops),
    }
    profiler = cProfile.Profile()
    started = time.perf_counter()
    row = profiler.runcall(pool_sweep.compute_domain, patch_id)
    wall = time.perf_counter() - started
    if pstats_path:
        profiler.dump_stats(pstats_path)
    stats = pstats.Stats(profiler)
    totals, builtin_split = classify(stats)
    grand = sum(totals.values())
    rows = []
    for func, (cc, nc, tt, ct, callers) in stats.stats.items():
        rows.append((tt, nc, func))
    rows.sort(key=lambda item: -item[0])
    top = []
    for tt, nc, (filename, lineno, name) in rows[:15]:
        tail = "/".join(filename.replace("\\", "/").split("/")[-2:])
        top.append(
            {
                "tottime": round(tt, 3),
                "share": round(tt / grand, 4),
                "ncalls": nc,
                "func": f"{tail}:{lineno}({name})",
                "bucket": bucket_of((filename, lineno, name)) or "builtin->callers",
            }
        )
    counters = row.get("prepared_counters", {})
    record = {
        "patch_id": patch_id,
        "profiled_wall_seconds": round(wall, 3),
        "unprofiled_seconds_in_row": row.get("seconds"),
        "profile_total_seconds": round(grand, 3),
        "outcome": row.get("outcome"),
        "work_spent": row.get("EXACT_WORK_SPENT"),
        "prepare_seconds_profiled": row.get("prepare_seconds"),
        "coverage_seconds_profiled": row.get("coverage_seconds"),
        "shape": shape,
        "counters": {
            key.replace("CONVEYOR_", ""): value
            for key, value in counters.items()
            if key
            in (
                "CONVEYOR_DOMAIN_EDGES",
                "CONVEYOR_SKELETON_NODES",
                "CONVEYOR_RATIONAL_VERTEX_FANS",
                "CONVEYOR_DEGRADED_MITER_CORNERS",
                "CONVEYOR_MITERED_CORNERS",
                "CONVEYOR_FACES",
                "CONVEYOR_ARRIVAL_LAWS",
                "CONVEYOR_LATTICE_SCALE",
            )
        },
        "bucket_seconds": {k: round(v, 3) for k, v in sorted(totals.items())},
        "bucket_share": {
            k: round(v / grand, 4) for k, v in sorted(totals.items())
        },
        "builtin_attribution": {
            name: {k: round(v, 3) for k, v in split.items()}
            for name, split in sorted(
                builtin_split.items(),
                key=lambda kv: -sum(kv[1].values()),
            )[:8]
        },
        "top15_tottime": top,
    }
    Path(out).write_text(
        json.dumps(record, ensure_ascii=False, indent=1), encoding="utf-8"
    )
    print(json.dumps({k: record[k] for k in (
        "patch_id", "profiled_wall_seconds", "bucket_share", "shape", "counters")},
        ensure_ascii=False))


if __name__ == "__main__":
    main()
