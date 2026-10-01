"""Где уходит время выгрузки САМЫХ тяжёлых доменов `building` (вне Blender).

Тяжёлые домены (patch 6, 1, 11, 7, 15) стоят в стене пула как выгрузка + счёт,
поэтому именно их выгрузка лежит на критическом пути: секунды стадий и
cProfile по каждому.

    python artifacts/host_export_parallel/heavy_export_probe.py [--prof] [patch ...]
"""

from __future__ import annotations

import cProfile
import io
import pstats
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "perf_prepare_diag"))

import env  # noqa: E402,F401
import big_scene  # noqa: E402

from cftuv.envelope_debug_profile import EnvelopeDebugProfileBuilderV1  # noqa: E402
from cftuv.envelope_metric_export import (  # noqa: E402
    build_envelope_patch_metric_export,
)
from cftuv.envelope_topology_export import (  # noqa: E402
    build_envelope_topology_export,
)


def main() -> None:
    patches = [int(item) for item in sys.argv[1:] if item.isdigit()] or [
        6, 1, 11, 7, 15,
    ]
    profiled = "--prof" in sys.argv
    _, bundle, _, _ = big_scene.survey()
    topology = build_envelope_topology_export(bundle)
    # Прогрев: импорты ядра и sympy не должны попадать в секунды первого домена.
    build_envelope_patch_metric_export(topology, 109)
    for patch_id in patches:
        profile = EnvelopeDebugProfileBuilderV1("heavy", "QUEUE")
        prof = cProfile.Profile()
        started = time.perf_counter()
        if profiled:
            prof.enable()
        build_envelope_patch_metric_export(topology, patch_id, profile=profile)
        if profiled:
            prof.disable()
        elapsed = time.perf_counter() - started
        totals = profile.snapshot().stage_totals
        print(
            f"patch {patch_id}: {elapsed:.3f} s  "
            + "  ".join(
                f"{name}={value:.3f}"
                for name, value in sorted(totals.items(), key=lambda item: -item[1])
                if value > 0.005
            )
        )
        if profiled:
            stream = io.StringIO()
            pstats.Stats(prof, stream=stream).sort_stats("tottime").print_stats(14)
            print(stream.getvalue()[:3800])


if __name__ == "__main__":
    main()
