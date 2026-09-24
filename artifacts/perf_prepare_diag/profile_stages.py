"""cProfile трёх стадий очереди по отдельности: PREPARE / COVERAGE / CONTOUR."""

from __future__ import annotations

import cProfile
import io
import pstats
import sys
import time

import env  # noqa: F401
from counterfactual import PATH, variants
from run_domain import bundle_from_field_snapshot, snapshot_and_request
from snapshot_bmesh import load_snapshot


def top(profile, tag, limit=40):
    stream = io.StringIO()
    stats = pstats.Stats(profile, stream=stream)
    stats.sort_stats("cumulative").print_stats(limit)
    cumulative = stream.getvalue()
    stream = io.StringIO()
    stats = pstats.Stats(profile, stream=stream)
    stats.sort_stats("tottime").print_stats(limit)
    return f"===== {tag} — CUMULATIVE =====\n{cumulative}\n===== {tag} — TOTTIME =====\n{stream.getvalue()}"


def main():
    density = int(sys.argv[1]) if len(sys.argv) > 1 else 0
    variant = sys.argv[2] if len(sys.argv) > 2 else "A_field"
    payload, cases = variants()
    selected = frozenset(payload["raw"]["selected_edges"])
    _, _, bundle = bundle_from_field_snapshot(PATH, vertex_override=cases[variant])
    rows = snapshot_and_request(bundle, selected, density=density)
    patch_id, domain_id, snapshot, request = rows[0]

    from cftuv_envelope.wavefront import conveyor_coverage, prepare_conveyor
    from cftuv.envelope_queue_export import build_queue_domain

    alpha_text = str(float(0.45))
    label = f"{variant}_d{density}"

    profile = cProfile.Profile()
    profile.enable()
    prepared = prepare_conveyor(snapshot, request)
    profile.disable()
    profile.dump_stats(str(env.OUT / f"prepare_{label}.pstats"))
    text = [top(profile, f"QUEUE_PREPARE {label} outcome={prepared.outcome.value}")]

    profile = cProfile.Profile()
    profile.enable()
    coverage = conveyor_coverage(prepared, alpha_text)
    profile.disable()
    profile.dump_stats(str(env.OUT / f"coverage_{label}.pstats"))
    text.append(top(profile, f"QUEUE_COVERAGE {label} outcome={coverage.outcome.value}"))

    profile = cProfile.Profile()
    profile.enable()
    domain = build_queue_domain(
        patch_id,
        domain_id,
        prepared,
        coverage,
        prepare_seconds=0.0,
        coverage_seconds=0.0,
    )
    profile.disable()
    profile.dump_stats(str(env.OUT / f"contour_{label}.pstats"))
    text.append(top(profile, f"QUEUE_CONTOUR {label} faces={len(domain.faces)}"))

    report = "\n".join(text)
    (env.OUT / f"profile_{label}.txt").write_text(report, encoding="utf-8")
    print(report[:200])
    print("written", env.OUT / f"profile_{label}.txt")


if __name__ == "__main__":
    main()
