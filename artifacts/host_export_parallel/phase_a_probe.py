"""Замер фазы A (выгрузка хоста по доменам) вне Blender: где уходят секунды.

Маршрут тот же, что у кнопки: `EnvelopeDebugSessionController` + провайдер
снапшота сессии + `_queue_snapshot_and_request` на КАЖДЫЙ домен, Fan Density 2.
Сцена — сохранённый снапшот `building` (`artifacts/perf_prepare_diag/big_scene`).

    python artifacts/host_export_parallel/phase_a_probe.py [--prof]
"""

from __future__ import annotations

import cProfile
import io
import pstats
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
DIAG = HERE.parent / "perf_prepare_diag"
sys.path.insert(0, str(DIAG))

import env  # noqa: E402,F401
import big_scene  # noqa: E402

from cftuv.envelope_debug_profile import EnvelopeDebugProfileBuilderV1  # noqa: E402
from cftuv.envelope_debug_session import EnvelopeDebugSessionController  # noqa: E402
from cftuv.envelope_queue_export import _queue_snapshot_and_request  # noqa: E402
from cftuv.envelope_request_export import (  # noqa: E402
    EnvelopeHostAdapterError,
    _typed_value,
)
from cftuv.envelope_topology_export import stage_domain_inputs  # noqa: E402


def main() -> None:
    _, bundle, selected, _ = big_scene.survey()
    controller = EnvelopeDebugSessionController()
    profile = EnvelopeDebugProfileBuilderV1("building", "QUEUE")
    started = time.perf_counter()
    topology_export = controller.get_topology_export(
        bundle, "obj", "data", profile=profile
    )
    print("topology_export", round(time.perf_counter() - started, 3))
    started = time.perf_counter()
    _, revision, patch_ids, request_id, by_domain = stage_domain_inputs(
        bundle, selected, profile=profile, topology_export=topology_export
    )
    print(
        "stage_domain_inputs",
        round(time.perf_counter() - started, 3),
        len(patch_ids),
    )

    def provider(patch_id, _domain_id):
        metric = controller.get_patch_metric(
            topology_export, patch_id, profile=profile
        )
        return controller.get_domain_geometry(metric, profile=profile).snapshot

    def phase_a():
        out = {}
        for patch_id in patch_ids:
            domain_id = _typed_value("patch-domain", revision, patch_id)
            try:
                out[domain_id] = _queue_snapshot_and_request(
                    bundle,
                    patch_id,
                    domain_id,
                    frozenset(by_domain[domain_id]),
                    0.45,
                    request_id,
                    density=2,
                    profile=profile,
                    topology_export=topology_export,
                    domain_snapshot_provider=provider,
                )
            except EnvelopeHostAdapterError as exc:
                out[domain_id] = exc
        return out

    prof = cProfile.Profile()
    started = time.perf_counter()
    profiled = "--prof" in sys.argv
    if profiled:
        prof.enable()
    result = phase_a()
    if profiled:
        prof.disable()
    print("phase A wall", round(time.perf_counter() - started, 3))
    totals = profile.snapshot().stage_totals
    for name, value in sorted(totals.items(), key=lambda item: -item[1]):
        print(f"  {name:28s} {value:8.3f}")
    print("refused", sum(isinstance(v, Exception) for v in result.values()))
    if profiled:
        for order, limit in (("cumulative", 50), ("tottime", 30)):
            stream = io.StringIO()
            pstats.Stats(prof, stream=stream).sort_stats(order).print_stats(
                limit
            )
            print(stream.getvalue()[:9000])


if __name__ == "__main__":
    main()
