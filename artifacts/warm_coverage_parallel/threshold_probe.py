"""Где пул покрытия начинает окупаться: партии из N самых малых доменов `building`.

`COVERAGE_POOL_MIN_BYTES` — порог, ниже которого партия считается в родителе.
Здесь он калибруется замером: для возрастающих партий (самые малые по размеру
пикла домены первыми) печатаются секунды в пуле (воркеры тёплые, пиклы сняты) и
в родителе, и суммарный размер пиклов партии.

    python artifacts/warm_coverage_parallel/threshold_probe.py [workers] [density]
"""

from __future__ import annotations

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
from cftuv.envelope_queue_export import recompute_queue_coverage  # noqa: E402
from cftuv.envelope_queue_pool import (  # noqa: E402
    COVERAGE_POOL_MIN_BYTES,
    SliderCoveragePool,
)


def main() -> None:
    workers = int(sys.argv[1]) if len(sys.argv) > 1 else 8
    density = int(sys.argv[2]) if len(sys.argv) > 2 else 2
    _, bundle, selected, _ = big_scene.survey()
    controller = EnvelopeDebugSessionController()
    evaluation = evaluate_envelope_debug_staged(
        bundle,
        selected,
        0.5,
        profile=EnvelopeDebugProfileBuilderV1("building", "QUEUE"),
        controller=controller,
        source_object_key="probe",
        source_data_key="probe",
        engine="QUEUE",
        density=density,
        workers=workers,
    )
    remember_queue_session(
        controller,
        "building",
        evaluation.topology_scene,
        evaluation.exact_debug_scenes,
        evaluation,
        density=density,
    )
    blobs = controller.preparation_blobs
    entries = sorted(
        controller.queue_session.entries, key=lambda item: len(blobs.blob_of(item[2]))
    )
    import cftuv.envelope_queue_pool as queue_pool

    queue_pool.COVERAGE_POOL_MIN_BYTES = 0  # замер: пул всегда
    pool = controller.slider_coverage_pool(workers, EnvelopeDebugProfileBuilderV1("b", "QUEUE"))
    assert pool is not None
    recompute_queue_coverage(entries[:8], "0.4", coverage_pool=pool)  # прогрев
    print(f"COVERAGE_POOL_MIN_BYTES = {COVERAGE_POOL_MIN_BYTES}")
    print("n  bytes     pool_s   sequential_s")
    for count in (1, 2, 4, 8, 16, 32, 64, 97, 114, 121):
        batch = entries[:count]
        total = sum(len(blobs.blob_of(item[2])) for item in batch)
        pooled = []
        sequential = []
        for _ in range(3):
            started = time.perf_counter()
            recompute_queue_coverage(batch, "0.45", coverage_pool=pool)
            pooled.append(time.perf_counter() - started)
            started = time.perf_counter()
            recompute_queue_coverage(batch, "0.45")
            sequential.append(time.perf_counter() - started)
        print(
            f"{count:3d} {total:8d}  {sorted(pooled)[1]:7.3f}  {sorted(sequential)[1]:7.3f}"
        )
    shutdown_domain_pool()


if __name__ == "__main__":
    main()
