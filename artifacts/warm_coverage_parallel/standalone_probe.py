"""Тёплая кнопка и ползунок на `building` вне Blender: секунды и равенство.

Холодная кнопка заполняет кэш подготовок, дальше: тёплая кнопка (покрытие
кэшированных подготовок — в пуле либо в родителе) и несколько шагов ползунка
alpha. Ответы пула и последовательного пути сравниваются по отпечатку без
секунд (`queue_scene_payload` без секундных ключей).

    python artifacts/warm_coverage_parallel/standalone_probe.py [workers] [density]
"""

from __future__ import annotations

import hashlib
import json
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
    build_queue_scene,
    queue_scene_payload,
    recompute_queue_coverage,
)

SECONDS = frozenset(
    {"prepare_seconds", "coverage_seconds", "contour_seconds", "timings"}
)


def fingerprint(scene) -> str:
    payload = queue_scene_payload(scene)
    for domain in payload["domains"]:
        for key in SECONDS:
            domain.pop(key, None)
    return hashlib.sha256(
        json.dumps(payload, sort_keys=True, default=str).encode()
    ).hexdigest()[:16]


def main() -> None:
    workers = int(sys.argv[1]) if len(sys.argv) > 1 else 8
    density = int(sys.argv[2]) if len(sys.argv) > 2 else 2
    _, bundle, selected, _ = big_scene.survey()
    controller = EnvelopeDebugSessionController()

    def press(alpha):
        profile = EnvelopeDebugProfileBuilderV1("building", "QUEUE")
        started = time.perf_counter()
        evaluation = evaluate_envelope_debug_staged(
            bundle,
            selected,
            alpha,
            profile=profile,
            controller=controller,
            source_object_key="probe",
            source_data_key="probe",
            engine="QUEUE",
            density=density,
            workers=workers,
        )
        wall = time.perf_counter() - started
        scene = remember_queue_session(
            controller,
            "building",
            evaluation.topology_scene,
            evaluation.exact_debug_scenes,
            evaluation,
            density=density,
        )
        snapshot = profile.snapshot()
        counters = {
            item.name: item.value
            for item in snapshot.counters
            if item.name.startswith("ENVELOPE_DOMAIN_POOL")
            and item.patch_domain_id is None
        }
        return wall, fingerprint(scene), counters, snapshot.stage_totals

    for label in ("cold", "warm", "warm", "warm"):
        wall, digest, counters, totals = press(0.5)
        print(
            f"{label:5s} press {wall:6.2f} s  digest {digest}  pool {counters}  "
            f"pool_wall {totals.get('QUEUE_POOL_WALL', 0.0):.2f}  "
            f"cover_sum {totals.get('QUEUE_COVERAGE', 0.0):.2f}"
        )

    entries = controller.queue_session.entries
    for alpha in ("0.375", "0.3", "0.45", "0.3"):
        profile = EnvelopeDebugProfileBuilderV1("building", "QUEUE")
        pool = controller.slider_coverage_pool(workers, profile)
        started = time.perf_counter()
        scene = recompute_queue_coverage(
            entries, alpha, coverage_pool=pool, profile=None
        )
        pooled = time.perf_counter() - started
        started = time.perf_counter()
        reference = recompute_queue_coverage(entries, alpha)
        sequential = time.perf_counter() - started
        print(
            f"slider {alpha}: pool {pooled:6.2f} s  sequential {sequential:6.2f} s  "
            f"equal {fingerprint(scene) == fingerprint(reference)}  "
            f"pool_used {pool is not None}"
        )
    shutdown_domain_pool()


if __name__ == "__main__":
    main()
