"""Порядок задач пула и потолок стены: что ставит кадр первым, и чем это платит.

Пул отдаёт воркерам задачи «тяжёлые первыми», а тяжесть оценивает размером
кадра. С выгрузкой в воркере кадр — лёгкий вход, не снапшот: ранжирование по
нему обязано совпасть с ранжированием по настоящему времени домена, иначе самый
тяжёлый домен встаёт позже и тянет за собой стену.

Скрипт берёт домены хранимой сцены `building`, считает размеры кадров (старого и
лёгкого), секунды из расписки `numeric_repr/baseline_18d7197.json` (d2, 8
воркеров) и прогоняет ЖАДНОЕ расписание на N воркерах для нескольких порядков:
настоящее время, размер снапшота, размер лёгкого входа (с коэффициентом и без).

    python artifacts/host_export_parallel/rank_probe.py [workers]
"""

from __future__ import annotations

import heapq
import json
import pickle
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path.insert(0, str(ROOT / "artifacts" / "perf_prepare_diag"))

import env  # noqa: E402,F401
import big_scene  # noqa: E402

from cftuv.envelope_domain_pool import EXPORT_FRAME_COST_SCALE  # noqa: E402
from cftuv.envelope_export_input import build_host_export_input  # noqa: E402
from cftuv.envelope_request_export import (  # noqa: E402
    EnvelopeHostAdapterError,
    _typed_value,
    build_envelope_analysis_snapshot,
    build_envelope_decal_request,
)
from cftuv.envelope_topology_export import (  # noqa: E402
    build_envelope_topology_export,
    stage_domain_inputs,
)


def makespan(costs: dict, order: list, workers: int, start: float = 0.0) -> float:
    free = [start] * workers
    heapq.heapify(free)
    end = 0.0
    for key in order:
        begin = heapq.heappop(free)
        finish = begin + costs[key]
        end = max(end, finish)
        heapq.heappush(free, finish)
    return end


def main() -> None:
    workers = int(sys.argv[1]) if len(sys.argv) > 1 else 8
    baseline = json.loads(
        (ROOT / "artifacts" / "numeric_repr" / "baseline_18d7197.json").read_text(
            encoding="utf-8"
        )
    )["runs"]["2"]["domains"]
    _, bundle, selected, _ = big_scene.survey()
    topology = build_envelope_topology_export(bundle)
    _, revision, patch_ids, request_id, by_domain = stage_domain_inputs(
        bundle, selected, topology_export=topology
    )
    rows = []
    for patch_id in patch_ids:
        domain_id = _typed_value("patch-domain", revision, patch_id)
        light = len(
            pickle.dumps(
                build_host_export_input(
                    topology,
                    patch_id,
                    alpha=0.45,
                    request_id=request_id,
                    density=2,
                ),
                5,
            )
        )
        try:
            snapshot = build_envelope_analysis_snapshot(
                bundle,
                included_patch_ids=frozenset({patch_id}),
                topology_export=topology,
            )
            request = build_envelope_decal_request(
                snapshot,
                frozenset(by_domain[domain_id]),
                0.45,
                decal_request_id_value=request_id,
                density=2,
            )
        except EnvelopeHostAdapterError:
            continue
        heavy = len(pickle.dumps((snapshot, request), 5))
        price = baseline[str(patch_id)]["price"]
        rows.append(
            {
                "patch": patch_id,
                "light": light,
                "snapshot": heavy,
                "export_s": price["host_export_seconds"],
                "solve_s": price["prepare_seconds"] + price["coverage_seconds"],
            }
        )
    total_light = sum(item["light"] for item in rows)
    total_heavy = sum(item["snapshot"] for item in rows)
    print(
        f"domains {len(rows)}; light {total_light} B, snapshot+request {total_heavy} B, "
        f"ratio {total_heavy / total_light:.2f}; scale in code {EXPORT_FRAME_COST_SCALE}"
    )
    by_time = sorted(rows, key=lambda item: -item["solve_s"])
    print("top 10 by real solve time:")
    for item in by_time[:10]:
        print(
            f"  patch {item['patch']:>3}  solve {item['solve_s']:7.2f}s  "
            f"export {item['export_s']:5.2f}s  light {item['light']:>7}  "
            f"snapshot {item['snapshot']:>7}"
        )
    orders = {
        "real time (ideal)": sorted(rows, key=lambda item: -item["solve_s"]),
        "snapshot size (old frames)": sorted(rows, key=lambda item: -item["snapshot"]),
        "light input size": sorted(rows, key=lambda item: -item["light"]),
        "real time + export (ideal, new)": sorted(
            rows, key=lambda item: -(item["solve_s"] + item["export_s"])
        ),
    }
    for name, ordered in orders.items():
        ranks = [item["patch"] for item in ordered[:8]]
        old_costs = {item["patch"]: item["solve_s"] for item in rows}
        new_costs = {
            item["patch"]: item["solve_s"] + item["export_s"] for item in rows
        }
        keys = [item["patch"] for item in ordered]
        print(
            f"{name:34s} top8 {ranks}  makespan old-work {makespan(old_costs, keys, workers):6.2f}s"
            f"  new-work {makespan(new_costs, keys, workers):6.2f}s"
        )


if __name__ == "__main__":
    main()
