"""Цена пересылки подготовки воркеру и цена покрытия по доменам `building` (вне Blender).

Отвечает на то, что надо знать до выбора схемы «покрытие закешированных
подготовок — в пул»: сколько весит пикл подготовки (всего и у тяжёлых), сколько
стоит `dumps`/`loads`, сколько стоит покрытие домена с хостовой записью (то, что
сейчас идёт в родителе), и каким получился бы идеальный потолок на N воркерах
при жадном расписании.

    python artifacts/warm_coverage_parallel/size_probe.py [workers] [density]
"""

from __future__ import annotations

import heapq
import json
import pickle
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path.insert(0, str(ROOT / "artifacts" / "perf_prepare_diag"))

import env  # noqa: E402,F401
import big_scene  # noqa: E402

from cftuv.envelope_debug_profile import EnvelopeDebugProfileBuilderV1  # noqa: E402
from cftuv.envelope_debug_session import (  # noqa: E402
    EnvelopeDebugSessionController,
    evaluate_envelope_debug_staged,
    remember_queue_session,
)
from cftuv.envelope_domain_pool import shutdown_domain_pool  # noqa: E402
from cftuv.envelope_queue_export import (  # noqa: E402
    load_queue_kernel,
    recompute_queue_coverage,
)


def makespan(costs: list, workers: int) -> float:
    free = [0.0] * workers
    heapq.heapify(free)
    end = 0.0
    for cost in sorted(costs, reverse=True):
        begin = heapq.heappop(free)
        finish = begin + cost
        end = max(end, finish)
        heapq.heappush(free, finish)
    return end


def main() -> None:
    workers = int(sys.argv[1]) if len(sys.argv) > 1 else 8
    density = int(sys.argv[2]) if len(sys.argv) > 2 else 2
    _, bundle, selected, _ = big_scene.survey()
    controller = EnvelopeDebugSessionController()
    profile = EnvelopeDebugProfileBuilderV1("building", "QUEUE")
    started = time.perf_counter()
    evaluation = evaluate_envelope_debug_staged(
        bundle,
        selected,
        0.5,
        profile=profile,
        controller=controller,
        source_object_key="size_probe",
        source_data_key="size_probe",
        engine="QUEUE",
        density=density,
        workers=workers,
    )
    print(f"cold press: {time.perf_counter() - started:.2f} s")
    remember_queue_session(
        controller,
        "building",
        evaluation.topology_scene,
        evaluation.exact_debug_scenes,
        evaluation,
        density=density,
    )
    entries = controller.queue_session.entries
    load_queue_kernel()
    rows = []
    for patch_id, domain_id, prepared in entries:
        started = time.perf_counter()
        blob = pickle.dumps(prepared, protocol=5)
        dump_seconds = time.perf_counter() - started
        started = time.perf_counter()
        pickle.loads(blob)
        load_seconds = time.perf_counter() - started
        started = time.perf_counter()
        recompute_queue_coverage(
            [(patch_id, domain_id, prepared)], "0.45", profile=None
        )
        coverage_seconds = time.perf_counter() - started
        # Второй и третий проход: память разложений тёплая, alpha другая.
        started = time.perf_counter()
        recompute_queue_coverage(
            [(patch_id, domain_id, prepared)], "0.3", profile=None
        )
        warm_seconds = time.perf_counter() - started
        started = time.perf_counter()
        recompute_queue_coverage(
            [(patch_id, domain_id, prepared)], "0.3", profile=None
        )
        warm_same_seconds = time.perf_counter() - started
        rows.append(
            {
                "patch": patch_id,
                "bytes": len(blob),
                "dump": dump_seconds,
                "load": load_seconds,
                "coverage_with_host": coverage_seconds,
                "warm_other_alpha": warm_seconds,
                "warm_same_alpha": warm_same_seconds,
            }
        )
    shutdown_domain_pool()
    rows.sort(key=lambda item: -item["coverage_with_host"])
    total = lambda key: sum(item[key] for item in rows)  # noqa: E731
    costs = [item["coverage_with_host"] for item in rows]
    summary = {
        "domains": len(rows),
        "blob_bytes_total": int(total("bytes")),
        "blob_bytes_max": max(item["bytes"] for item in rows),
        "dump_total_s": round(total("dump"), 3),
        "load_total_s": round(total("load"), 3),
        "coverage_with_host_total_s": round(total("coverage_with_host"), 3),
        "coverage_with_host_max_s": round(max(costs), 3),
        "warm_other_alpha_total_s": round(total("warm_other_alpha"), 3),
        "warm_other_alpha_max_s": round(max(item["warm_other_alpha"] for item in rows), 3),
        "warm_same_alpha_total_s": round(total("warm_same_alpha"), 3),
        "warm_same_alpha_max_s": round(max(item["warm_same_alpha"] for item in rows), 3),
        "warm_makespan_8_s": round(makespan([item["warm_same_alpha"] for item in rows], 8), 3),
        "ideal_makespan_s": {
            str(n): round(makespan(costs, n), 3) for n in (2, 4, 8, 12)
        },
        "ideal_makespan_with_load_s": {
            str(n): round(
                makespan([c + r["load"] for c, r in zip(costs, rows)], n), 3
            )
            for n in (8,)
        },
        "top": [
            {key: (round(value, 4) if isinstance(value, float) else value) for key, value in item.items()}
            for item in rows[:10]
        ],
        "median_coverage_s": round(sorted(costs)[len(costs) // 2], 4),
    }
    buckets = {}
    for limit in (8_000, 16_000, 32_000, 64_000, 128_000, 10**9):
        part = [item for item in rows if item["bytes"] < limit]
        buckets[str(limit)] = {
            "domains": len(part),
            "warm_cost_s": round(sum(item["warm_same_alpha"] for item in part), 3),
            "cold_cost_s": round(sum(item["coverage_with_host"] for item in part), 3),
        }
    summary["by_blob_size_below"] = buckets
    print(json.dumps(summary, indent=1))


if __name__ == "__main__":
    main()
