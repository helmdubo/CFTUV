"""blender_live_drag: a scripted width drag on the live path with the real modal session functions, headless (LIVE_DRAG_STREAM_V1).

    blender -b scene.blend --python-exit-code 1 --python tools/blender_live_drag.py -- --root <tree> --native-path <dir> --case building \
        --workers 12 --live on|off --steps 120 --interval 0.0167 --span 0.10 --pause 0.4 --out drag.json

WHAT IT DOES. The cold button (the operator), then the drag exactly as `Adjust Decal Width` runs it: `begin_adjust` (the drag starts, the live-drag log is opened when the switch is on),
per mouse step the two calls of `apply_step` (`width_event_preview`: the model frame; `request_drag_tick`: the exact tick order), the confirm `finish_adjust` (the one write of the
width, the COMMIT order) and the drain of the scheduler. There is no event loop in a headless Blender, so the tool is the main loop: between the mouse steps it calls the scheduler's
`step` whenever its timer would fire, and sleeps otherwise (the compute thread runs meanwhile, as it does behind the Blender UI). The path: `--steps` mouse steps at `--interval`
seconds, from the cold width up by `--span`, down to minus `--span`, back to `+--release` (a share of the cold width, the release width), with a pause of `--pause` seconds at each turn
(the hand stops: the idle reconcile has its chance). `--live off` is the product default (and the only mode a tree without the switch has).

WHAT IT REPORTS (one JSON; last line LIVE_DRAG_OK). Per step: the width, the main-thread cost of the event (model frame + order), whether the step's own exact tick landed before the next
step (the drawn mesh was EXACT at that width) or not (MODEL), the latency of its own exact tick when it landed at all, and the catch-up latency (until an exact tick of this or a later width
landed). Totals: p50/p90 of the latencies, the share of exact steps, the live-drag log (`envelope_width_stream.report`: superseded, nearest, preempted, reconciles and what they corrected,
deferred verifications), the commit wall (release -> committed mesh) and the committed mesh digest (`mesh_content_digest`) and arrays hash: two runs that release at the same width must
give the same committed mesh, whatever the switch. The scene is never saved and no Blender but this one is touched.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
import traceback
from collections import Counter
from pathlib import Path
from types import SimpleNamespace

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import blender_drag_ticks as drag_ticks  # noqa: E402  (the tree loader, the session after the cold button, the mesh hash and the percentiles are shared)


def _arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", required=True)
    parser.add_argument("--native-path", default="")
    parser.add_argument("--case", default="building")
    parser.add_argument("--workers", type=int, default=12)
    parser.add_argument("--live", default="on", choices=("on", "off", "ticks"))
    parser.add_argument("--settle", default="on", choices=("on", "off"), help="LIVE_DRAG_SETTLE_ON_PAUSE_V1 (\"Settle topology on pause\") under --live on")
    parser.add_argument("--steps", type=int, default=120)
    parser.add_argument("--interval", type=float, default=1.0 / 60.0, help="seconds between mouse steps (60 Hz)")
    parser.add_argument("--span", type=float, default=0.10, help="the drag reaches +-span of the cold width")
    parser.add_argument("--release", type=float, default=0.03, help="the release width, as a share of the cold width above it")
    parser.add_argument("--pause", type=float, default=0.4, help="seconds the hand rests at each turn of the path")
    parser.add_argument("--dwell", type=float, default=0.05, help="seconds between the last step and the release")
    parser.add_argument("--kernel-backend", default="NATIVE")
    parser.add_argument("--out", required=True)
    return parser.parse_args(sys.argv[sys.argv.index("--") + 1 :] if "--" in sys.argv else [])


class _NoTimers:
    """The tool is the main loop: the scheduler's timer is stepped by hand, never registered."""

    def register(self, function, first_interval):
        return None

    def is_registered(self, function):
        return True


def drag_path(base: float, steps: int, span: float, release: float) -> list:
    """`(width, pause_before)` per mouse step: up to +span, down to -span, back to +release; a pause at the two turns."""

    legs = [(base, base * (1 + span), steps // 4), (base * (1 + span), base * (1 - span), steps // 2), (base * (1 - span), base * (1 + release), steps - steps // 4 - steps // 2)]
    path = []
    for number, (start, end, count) in enumerate(legs):
        for index in range(count):
            path.append((start + (end - start) * (index + 1) / count, number > 0 and index == 0))
    return path


def drive_until(scheduler, deadline: float) -> None:
    """The main loop between two events: the scheduler's step whenever it asks for one, sleep otherwise."""

    due = time.perf_counter()
    while True:
        now = time.perf_counter()
        if now >= deadline:
            return
        if scheduler is not None and now >= due:
            delay = scheduler.step()
            due = now + (delay if delay is not None else 0.005)
        time.sleep(max(0.0005, min(deadline, due) - time.perf_counter()))


def step_metrics(events, release_at: float) -> list:
    """Per step: did its own exact tick land before the next step, its own latency, the catch-up latency (an exact tick of this or a later width)."""

    steps = [item for item in events if item[0] == "step"]
    exact = sorted((item for item in events if item[0] == "exact"), key=lambda item: item[3])
    rows = []
    for number, (_kind, sequence, width, moment) in enumerate(steps):
        until = steps[number + 1][3] if number + 1 < len(steps) else release_at
        own = next((item for item in exact if item[1] == sequence), None)
        caught = next((item for item in exact if item[1] >= sequence and item[3] >= moment), None)
        shown = [item for item in exact if item[3] <= until]  # the newest exact width on screen just before the next step (the model moves it to the pointer)
        rows.append({
            "width": width, "at": moment, "exact": own is not None and own[3] <= until, "own_latency": None if own is None else own[3] - moment,
            "catch_up": None if caught is None else caught[3] - moment, "caught_before_release": caught is not None and caught[3] <= release_at,
            "exact_width_gap": None if not shown else abs(width - shown[-1][2]),
        })
    return rows


def run_drag(session, args, report: dict) -> None:
    import bpy

    import cftuv.envelope_width_live as live
    from cftuv.envelope_width_adjust import PHASE_CONFIRMED
    from cftuv.envelope_width_session import ViewScaleV1, begin_adjust, finish_adjust

    try:
        from cftuv.envelope_width_stream import report as stream_report
    except ImportError:  # a tree without LIVE_DRAG_STREAM_V1 (the base of the A/B): no live-drag log
        stream_report = None

    controller, context = session.controller, bpy.context
    if args.live == "on" and not hasattr(session.mesh_settings, "drag_live_mesh"):
        raise RuntimeError("this tree has no LIVE_DRAG_STREAM_V1 switch (drag_live_mesh)")
    if hasattr(session.mesh_settings, "drag_live_mesh"):
        session.mesh_settings.drag_live_mesh = args.live == "on"
    if hasattr(session.mesh_settings, "drag_settle_on_pause"):
        session.mesh_settings.drag_settle_on_pause = args.settle == "on"
    session.mesh_settings.drag_exact_ticks = args.live == "ticks"  # the previous switch "Exact ticks while dragging" (debounced, cancelled by the hand)
    scheduler = live.scheduler_of(controller)
    scheduler._timers = _NoTimers()
    scheduler._clock = time.perf_counter  # Windows `time.monotonic` ticks in 15.6 ms: the orders, the job moments and the tool share the fine counter
    events: list = []
    original_apply = live._apply
    laps = drag_ticks.Laps()
    report["laps_found"] = drag_ticks.install_laps(laps)
    ticks: list = []
    report["ticks"] = ticks

    def recorded_apply(session_controller, request, job, value):  # the scheduler looks `_apply` up by module global: every applied exact result is recorded here, whatever the mode
        applying = time.perf_counter()
        original_apply(session_controller, request, job, value)
        if getattr(request.payload, "grade", "COMMIT") != "COMMIT":
            events.append(("exact", request.sequence, request.alpha, time.perf_counter(), request.requested_at, getattr(job, "started_at", 0.0) or applying, getattr(job, "finished_at", 0.0) or applying, applying))
            plan = getattr(value, "plan", None)
            held = {} if plan is None else dict(Counter((plan.held or {}).values()))
            ticks.append({  # per applied tick: its decision and the parent laps since the previous apply (the ticks are sequential: one flight)
                "sequence": request.sequence, "alpha": request.alpha, "compute": float(getattr(job, "seconds", 0.0)), "laps": laps.snapshot(), "run_wall": float(getattr(value.run, "wall_seconds", 0.0)),
                "plan": None if plan is None else {"mode": plan.mode, "reason": plan.reason, "served": len(plan.served or ()), "held": held, "composite": plan.composite,
                                                   "fallback": plan.fallback, "deferred": getattr(plan, "deferred", ""), "spliced": getattr(plan, "spliced", "")},
            })
        laps.reset()

    live._apply = recorded_apply
    base = float(session.settings.envelope_debug_alpha)
    report["cold_width"] = base
    path = drag_path(base, args.steps, args.span, args.release)
    runtime = begin_adjust(context, (0.0, 0.0), ViewScaleV1((0.0, 0.0), 1e-3))
    if isinstance(runtime, str):
        raise RuntimeError(f"the drag did not start: {runtime}")
    costs, samples, clock = [], [], time.perf_counter()
    for width, pause in path:
        clock += args.interval + (args.pause if pause else 0.0)
        drive_until(scheduler, clock)
        started = time.perf_counter()
        live.width_event_preview(controller, width, runtime.offset)
        if live.request_drag_tick(context, controller, width):
            events.append(("step", scheduler._sequence, width, time.perf_counter()))
        else:
            events.append(("step", -1, width, time.perf_counter()))
        costs.append(time.perf_counter() - started)
        samples.append((clock, started, costs[-1]))  # when the event was due, when the main loop got to it (late: a step of the scheduler or waiting for the GIL), its cost
    drive_until(scheduler, time.perf_counter() + args.dwell)
    release_at = time.perf_counter()
    report["release_at"] = release_at
    log = getattr(controller, "width_live_drag", None)
    if log is not None and controller.width_displayed is not None:
        from cftuv.envelope_width_stream import unverified_at

        report["unverified_at_release"] = unverified_at(log, controller.width_displayed, log.cursor)
    report["drag_log"] = None if log is None else stream_report(log, scheduler.counters)
    report["scheduler_at_release"] = str(scheduler.counters)
    events = sorted(events, key=lambda item: item[3])
    finished = SimpleNamespace(session=SimpleNamespace(width=path[-1][0], phase=PHASE_CONFIRMED), controller=controller, start_width=base, offset=runtime.offset)
    report["counters_at_release"] = str(scheduler.counters)
    assert finish_adjust(context, finished, confirmed=True) == "FINISHED"
    commit_sequence = scheduler._sequence  # the COMMIT order is the last order of the drag
    scheduler.settle()
    report["counters_after_commit"] = str(scheduler.counters)
    if scheduler.last_applied is None or scheduler.last_applied.sequence != commit_sequence:
        raise RuntimeError(f"the commit did not land: {scheduler.counters} {scheduler.last_applied}")
    # The width property is single precision: the committed width is the release width as the property holds it.
    report["commit"] = {"width": scheduler.last_applied.alpha, "release_width": path[-1][0], "wall": time.perf_counter() - release_at, "latency": scheduler.last_applied.latency_seconds, "compute": scheduler.last_applied.compute_seconds,
                        "apply": scheduler.last_applied.apply_seconds}
    live._apply = original_apply
    rows = step_metrics([item for item in events if item[0] != "exact" or item[3] <= release_at], release_at)
    report["exact_events"] = [list(item) for item in events if item[0] == "exact"]
    report["steps"], report["event_cost"] = rows, drag_ticks.stats(costs)
    report["event_samples"] = samples
    report["event_late"] = drag_ticks.stats([started - due for due, started, _cost in samples])
    summarize(report, rows, log)


def summarize(report: dict, rows: list, log) -> None:
    own = [row["own_latency"] for row in rows if row["own_latency"] is not None]
    caught = [row["catch_up"] for row in rows if row["caught_before_release"]]
    base = float(report["cold_width"])
    gaps = [100.0 * row["exact_width_gap"] / base for row in rows if row["exact_width_gap"] is not None]
    ticks = [item for item in report.get("exact_events", []) if item[3] <= report["release_at"]]
    report["latency_parts"] = {  # order -> start (one flight: waiting for the tick in flight), start -> end (the tick itself), end -> apply (the poll), the apply on the main thread
        "queue": drag_ticks.stats([item[5] - item[4] for item in ticks]), "compute": drag_ticks.stats([item[6] - item[5] for item in ticks]),
        "poll": drag_ticks.stats([item[7] - item[6] for item in ticks]), "apply": drag_ticks.stats([item[3] - item[7] for item in ticks]),
    }
    report["summary"] = {
        "exact_width_gap_percent": drag_ticks.stats(gaps),
        "verify_domains": None if log is None else log.verify_domains, "idle_ticks": None if log is None else log.idle_ticks,
        "steps": len(rows) or report["args"]["steps"], "exact_steps": sum(1 for row in rows if row["exact"]), "model_steps": sum(1 for row in rows if not row["exact"]) if rows else report["args"]["steps"],
        "exact_share": round(sum(1 for row in rows if row["exact"]) / len(rows), 4) if rows else 0.0,
        "own_latency": drag_ticks.stats(own), "catch_up_latency": drag_ticks.stats(caught), "steps_with_own_tick": len(own), "steps_caught_up_before_release": len(caught),
        "structure_changes_corrected": None if log is None else log.structure_changes, "layout_changes_corrected": None if log is None else log.layout_changes,
        "full_ticks": None if log is None else dict(log.full_ticks), "unverified_at_release": report.get("unverified_at_release"),
    }
    report["settle"] = settle_metrics(report, rows)


#: The hand "moves" while the previous width step is younger than this (LIVE_DRAG_SETTLE_ON_PAUSE_V1 uses the same 0.12 s); a step followed by a gap longer than PAUSE_GAP starts a pause.
MOVING_SECONDS = 0.12
PAUSE_GAP = 0.5


def settle_metrics(report: dict, rows: list) -> dict:
    """Swaps during motion (an exact tick applied while the previous step is younger than MOVING_SECONDS and the hand moves on), the settle latency (the last step before a pause or the release ->
    an exact mesh of that or a later width) and the mouse events split: while moving, and the first events after a pause (the hand resumes while the settled swap may still be applying)."""

    steps = [row["at"] for row in rows]
    applies = [item[3] for item in report.get("exact_events", []) if item[3] <= report["release_at"]]
    swaps_moving = 0
    for moment in applies:
        before = [at for at in steps if at <= moment]
        after = [at for at in steps if at > moment]
        if before and moment - before[-1] < MOVING_SECONDS and after and after[0] - before[-1] < PAUSE_GAP:
            swaps_moving += 1
    pauses = [index for index, at in enumerate(steps) if index + 1 == len(steps) or steps[index + 1] - at > PAUSE_GAP]
    settle = [rows[index]["catch_up"] for index in pauses if rows[index]["catch_up"] is not None]
    samples = report.get("event_samples", [])
    resumed = {index + 1 for index in pauses if index + 1 < len(samples)}
    moving = [item for index, item in enumerate(samples) if index not in resumed]
    first = [item for index, item in enumerate(samples) if index in resumed]
    return {
        "swaps_during_motion": swaps_moving, "swaps_total": len(applies), "pauses": len(pauses), "settle_latency": drag_ticks.stats(settle),
        "moving_event_cost": drag_ticks.stats([item[2] for item in moving]), "moving_event_late": drag_ticks.stats([item[1] - item[0] for item in moving]),
        "resume_event_cost": drag_ticks.stats([item[2] for item in first]), "resume_event_late": drag_ticks.stats([item[1] - item[0] for item in first]),
    }


def main() -> int:
    args = _arguments()
    args.grade, args.ticks, args.confirms = "commit", 0, 0  # the Session of blender_drag_ticks reads these names; they mean nothing here
    report: dict = {"schema": "cftuv.live-drag.v1", "args": vars(args), "failure": None}
    try:
        session = drag_ticks.Session(args)
        report["native_build_id"] = session.native["build_id"]
        report["cold_press_s"] = round(session.cold_press(), 4)
        if session.bpy.context.mode != "OBJECT":
            session.bpy.ops.object.mode_set(mode="OBJECT")
        report["cold_mesh_digest"] = session.mesh_module.mesh_content_digest(session.decal_mesh)
        run_drag(session, args, report)
        report["final_mesh_digest"] = session.mesh_module.mesh_content_digest(session.decal_mesh)
        report["final_arrays_hash"] = drag_ticks.mesh_arrays_hash(session.decal_mesh)
    except Exception:  # noqa: BLE001 - the traceback is the report
        report["failure"] = traceback.format_exc()
        print(report["failure"])
    try:
        from cftuv.envelope_domain_pool import shutdown_domain_pool

        shutdown_domain_pool()
    except Exception as exc:  # noqa: BLE001 - the cleanup does not change the verdict
        print("pool shutdown:", type(exc).__name__, exc)
    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    Path(args.out).write_text(json.dumps(report, ensure_ascii=False, indent=1, sort_keys=True, default=str) + "\n", encoding="utf-8")
    if report["failure"] is None:
        summary = report["summary"]
        print(f"LIVE_DRAG exact share {summary['exact_share']} own p50 {summary['own_latency'].get('p50')} p90 {summary['own_latency'].get('p90')} "
              f"verified {summary['verify_domains']} commit {report['commit']['wall']:.3f} s digest {report['final_mesh_digest'][:16]}")
    print("LIVE_DRAG_OK" if report["failure"] is None else "LIVE_DRAG_FAILED")
    return 0 if report["failure"] is None else 1


if __name__ == "__main__":
    code = main()
    if code:
        raise SystemExit(code)
