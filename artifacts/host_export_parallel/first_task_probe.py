"""Цена ПЕРВОЙ задачи воркера после «готов»: что подгружается лениво.

Воркер поднимает ядро и модули выгрузки до «готов», но sympy и часть ядра
подгружаются изнутри функций; кто платит за них — первая задача каждого воркера,
то есть критический путь стены. Скрипт повторяет старт воркера (`load_queue_kernel`
и `load_export_modules`), затем исполняет ту же выгрузку дважды и печатает секунды
обоих исполнений и модули, появившиеся при первом.

    python artifacts/host_export_parallel/first_task_probe.py [patch]
"""

from __future__ import annotations

import pickle
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path.insert(0, str(ROOT / "artifacts" / "perf_prepare_diag"))

import env  # noqa: E402,F401
import big_scene  # noqa: E402

from cftuv.envelope_domain_pool import DomainTaskV1, solve_task  # noqa: E402
from cftuv.envelope_export_input import (  # noqa: E402
    build_host_export_input,
    load_export_modules,
)
from cftuv.envelope_queue_export import load_queue_kernel  # noqa: E402
from cftuv.envelope_request_export import _typed_value  # noqa: E402
from cftuv.envelope_topology_export import (  # noqa: E402
    build_envelope_topology_export,
    stage_domain_inputs,
)


def main() -> None:
    patch_id = int(sys.argv[1]) if len(sys.argv) > 1 else 109
    _, bundle, selected, _ = big_scene.survey()
    topology = build_envelope_topology_export(bundle)
    _, revision, _, request_id, by_domain = stage_domain_inputs(
        bundle, selected, topology_export=topology
    )
    domain_id = _typed_value("patch-domain", revision, patch_id)
    export = build_host_export_input(
        topology, patch_id, alpha=0.45, request_id=request_id, density=2
    )
    export = pickle.loads(pickle.dumps(export, 5))
    started = time.perf_counter()
    load_queue_kernel()
    load_export_modules()
    print(f"worker preload: {time.perf_counter() - started:.2f} s")
    before = set(sys.modules)

    def task():
        return DomainTaskV1(
            0,
            patch_id,
            domain_id,
            None,
            None,
            "0.45",
            frozenset(by_domain[domain_id]),
            export,
        )

    for attempt in ("first", "second"):
        started = time.perf_counter()
        result = solve_task(task())
        elapsed = time.perf_counter() - started
        timings = {
            item.stage: round(item.elapsed_seconds, 3)
            for item in result.export_timings
        }
        print(
            f"{attempt}: {elapsed:.3f} s ok={result.ok} export stages {timings}"
        )
        if attempt == "first":
            late = sorted(set(sys.modules) - before)
            print(f"modules imported by the first task: {len(late)}")
            for name in late[:60]:
                print("   ", name)


if __name__ == "__main__":
    main()
