"""Interval linearity and width-independence probe (headless Blender only; the scene is never saved).

    blender -b <scene> --python-exit-code 1 --python field_linearity.py -- --root <tree> --native-path <site> --case building|cover008 [--stride N] --out out.json

One cold press in this process (workers=0) with a recording wrapper around `materialize.step.step_domain` (the step of the width tick) and around
`materialize.interval.event_table` (the alpha-free event table of each preparation). Then per captured materialized domain:
  * EVENTS: the event lows (`EventTableV1.lo`, metres of width: front arrivals at skeleton vertices and clip crossings) inside drag ranges above and below alpha0;
  * INSIDE THE INTERVAL: exact steps (parametric_exact) at 3 widths above and 3 below alpha0 inside the domain's certified interval (capped at +-25 %):
      - the theorem of AFFINE_DRAG_CERTIFICATE_V1 by EXACT SqrtSumV1 equality (kernel/tests/parametric_theorem.py): map points, lifted positions, (s, r) UV facts and their slopes;
      - the structure signature (rings = tessellation/fans/caps, chains = seams/interfaces, station facts, regions, diagnostics) and which parts differ;
      - the binary64 slopes of the certificate (bytes) and the float error of value + slope * delta against the real batch;
  * ACROSS: one width 5 % above alpha0 (usually past an event), the same comparisons (sanity: events do change structure).
  * TIMING: the product step at alpha0 with and without the certificate (min of 2), the share of the tick a cache of slopes would save.
"""
from __future__ import annotations

import argparse
import json
import sys
import time
import traceback
from collections import Counter
from fractions import Fraction
from pathlib import Path

sys.dont_write_bytecode = True

CASES = {"building": ("testscene", "building", 0.2239, "2", 42, -1.0), "cover008": ("cover008", "cover.008", 0.04214220866560936, "2", 20, 0.004999999888241291)}
RANGES = (0.005, 0.01, 0.02, 0.05, 0.10, 0.25)


def main() -> int:
    import bpy

    argv = sys.argv[sys.argv.index("--") + 1:]
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", required=True)
    parser.add_argument("--native-path", required=True)
    parser.add_argument("--case", required=True)
    parser.add_argument("--workers", type=int, default=0)
    parser.add_argument("--stride", type=int, default=1)
    parser.add_argument("--focus", default="")
    parser.add_argument("--focus-high", default="0.05004387279")
    parser.add_argument("--out", required=True)
    args = parser.parse_args(argv)
    root = Path(args.root).resolve()
    sys.path.insert(0, str(root / "tools"))
    sys.path.insert(0, str(root / "kernel" / "tests"))
    import blender_tree_ab as T

    T.load_tree(root, args.native_path)
    import bmesh
    import cftuv.envelope_production_export as export_module
    import cftuv_envelope.materialize.interval as interval_module
    import cftuv_envelope.materialize.step as step_module
    import parametric_theorem as theorem
    from cftuv.analysis import build_analysis_bundle
    from cftuv.analysis_surface import source_revision_from_bmesh
    from cftuv.envelope_debug_session import WINDOW_MANAGER_SESSION_ATTRIBUTE, EnvelopeDebugSessionController
    from cftuv.envelope_gc_pause import gc_paused
    from cftuv.envelope_kernel_backend import kernel_backend_of
    from cftuv.envelope_request_policy import envelope_dissolve_uv_slide, envelope_stretch_budget

    scene_kind, mesh, alpha, density, stretch, offset = CASES[args.case]
    report: dict = {"case": args.case, "alpha0": alpha, "blend": bpy.data.filepath, "failure": None}
    captured: list = []
    tables: dict = {}
    real_step = step_module.step_domain
    real_table = interval_module.event_table
    try:
        report["native"] = T.verify_native(args.native_path, "")
        scene, obj, settings, mesh_settings = T._open_case(argparse.Namespace(scene_kind=scene_kind, mesh=mesh))
        mesh_settings.kernel_backend = "NATIVE"
        if offset >= 0:
            mesh_settings.offset = offset
        controller = EnvelopeDebugSessionController()
        setattr(bpy.context.window_manager, WINDOW_MANAGER_SESSION_ATTRIBUTE, controller)
        selected = T.select_edges(obj, "stored" if scene_kind == "cover008" else "seam")
        source_bm = bmesh.from_edit_mesh(obj.data)
        source_bm.faces.ensure_lookup_table()
        face_indices = tuple(face.index for face in source_bm.faces)
        revision = source_revision_from_bmesh(source_bm, obj, face_indices)
        key, data_key = int(obj.as_pointer()), int(obj.data.as_pointer())
        bundle = controller.get_analysis_bundle(key, data_key, revision, lambda: build_analysis_bundle(source_bm, face_indices, obj))

        def recording(prepared, alpha_text, **kw):
            found = real_step(prepared, alpha_text, **kw)
            captured.append((prepared, alpha_text, {k: v for k, v in kw.items() if k not in ("parametric", "parametric_exact")}, found))
            return found

        def table_recording(prepared, triangles, lift_key):
            table = real_table(prepared, triangles, lift_key)
            tables[id(prepared)] = table
            return table

        step_module.step_domain = recording
        interval_module.event_table = table_recording

        def press(width):
            with gc_paused():
                return export_module.run_production(
                    controller, bundle, frozenset(selected), width, source_object_key=key, source_data_key=data_key, density=density,
                    developable_stretch_budget=envelope_stretch_budget(stretch), silhouette_uv_slide=envelope_dissolve_uv_slide(settings.envelope_debug_dissolve_uv_tolerance),
                    workers=0, kernel_backend=kernel_backend_of(mesh_settings),
                )

        started = time.perf_counter()
        run = press(alpha)
        report["press_seconds"] = time.perf_counter() - started
        step_module.step_domain = real_step
        report["domains"] = len(run.results)
        report["captured"] = len(captured)
        print(f"LIN press domains={len(run.results)} captured={len(captured)} {report['press_seconds']:.1f}s", flush=True)

        counts: Counter = Counter()
        part_changes: Counter = Counter()
        failures, per_domain = [], []
        events_up = {r: [] for r in RANGES}
        events_down = {r: [] for r in RANGES}
        global_up = {r: set() for r in RANGES}
        global_down = {r: set() for r in RANGES}
        float_worst = [0.0, 0.0]
        time_with = time_without = 0.0
        alpha0 = Fraction(theorem.decimal_text(Fraction(alpha)))
        seen = set()
        domains = [item for item in captured if item[3].result.is_materialized]
        for number, (prepared, alpha_text, kw, first) in enumerate(domains):
            if id(prepared) in seen:
                continue
            seen.add(id(prepared))
            patch = first.result.batch.patch_domain_id.value
            table = tables.get(id(prepared))
            row = {"patch": patch, "interval": None if first.result.interval is None else first.result.interval.as_record()}
            if table is not None and not table.refusal:
                lows = table.lo
                a = float(alpha_text)
                for r in RANGES:
                    up = [x for x in lows if a < x <= a * (1 + r)]
                    down = [x for x in lows if a * (1 - r) <= x < a]
                    events_up[r].append(len(up))
                    events_down[r].append(len(down))
                    global_up[r].update(up)
                    global_down[r].update(down)
                row["events_total"] = len(lows)
            else:
                counts["NO_EVENT_TABLE" if table is None else "EVENT_TABLE_REFUSED:" + table.refusal] += 1
            if number % args.stride:
                per_domain.append(row)
                continue
            kw = {**kw, "digests": False}
            # timing of the product step at alpha0: with and without the slope certificate (the tick computes it every time)
            best_with = best_without = None
            for _ in range(2):
                t0 = time.perf_counter(); real_step(prepared, alpha_text, **kw, parametric=True); t1 = time.perf_counter()
                real_step(prepared, alpha_text, **kw, parametric=False); t2 = time.perf_counter()
                best_with = t1 - t0 if best_with is None else min(best_with, t1 - t0)
                best_without = t2 - t1 if best_without is None else min(best_without, t2 - t1)
            time_with += best_with
            time_without += best_without
            base = real_step(prepared, alpha_text, **kw, parametric=True, parametric_exact=True)
            certificate = base.result.parametric
            interval = base.result.interval
            row["markers"] = list(certificate.markers) if certificate is not None else ["PARAM_ABSENT"]
            row["approximate"] = list(certificate.approximate) if certificate is not None else []
            if not theorem.usable(certificate) or interval is None or interval.status != "CERTIFIED":
                counts["BASE_NOT_USABLE"] += 1
                per_domain.append(row)
                continue
            a0 = Fraction(alpha_text)
            hi = Fraction(interval.high) if interval.high is not None else a0 * Fraction(5, 4)
            hi = min(hi, a0 * Fraction(5, 4))
            lo = max(Fraction(interval.low), a0 * Fraction(3, 4))
            samples = [a0 + (hi - a0) * Fraction(k, 4) for k in (1, 2, 3)] + [a0 - (a0 - lo) * Fraction(k, 4) for k in (1, 2, 3)]
            samples = [Fraction(theorem.decimal_text(s)) for s in samples]
            row["samples"] = []
            for s in samples + [Fraction(theorem.decimal_text(a0 * Fraction(21, 20)))]:  # the width as the kernel receives it (decimal text), so the theorem compares the same number
                inside = interval.contains_exact(s)
                other = real_step(prepared, theorem.decimal_text(s), **kw, parametric=True, parametric_exact=True)
                tag = "INSIDE" if inside else "OUTSIDE"
                if not other.result.is_materialized:
                    counts["OTHER_REFUSED_" + tag] += 1
                    continue
                differing = base.result.structure.differing(other.result.structure)
                for name in differing:
                    part_changes[f"{tag}:{name}"] += 1
                slopes_same = None
                oc = other.result.parametric
                if oc is not None and not oc.markers and certificate.position_slopes is not None and oc.position_slopes is not None:
                    slopes_same = bytes(memoryview(certificate.position_slopes)) == bytes(memoryview(oc.position_slopes)) and bytes(memoryview(certificate.loop_slopes)) == bytes(memoryview(oc.loop_slopes))
                outcome, found = theorem.theorem_differences(base, other, a0, s)
                counts[f"{outcome}_{tag}"] += 1
                counts[f"STRUCTURE_{'SAME' if not differing else 'CHANGED'}_{tag}"] += 1
                if slopes_same is not None:
                    counts[f"FLOAT_SLOPES_{'EQUAL' if slopes_same else 'DIFFER'}_{tag}"] += 1
                if outcome == theorem.JUDGED:
                    counts[f"THEOREM_{'HOLDS' if not found else 'FAILS'}_{tag}"] += 1
                    if found:
                        failures.append({"patch": patch, "alpha": str(s), "inside": inside, "differences": [str(x) for x in found[:5]], "count": len(found)})
                    if inside:
                        errors = theorem.float_errors(base, other, a0, s)
                        if errors is not None:
                            float_worst[0] = max(float_worst[0], errors[0]); float_worst[1] = max(float_worst[1], errors[1])
                row["samples"].append({"alpha": float(s), "inside": inside, "outcome": outcome, "differences": len(found), "structure_parts_changed": list(differing), "float_slopes_equal": slopes_same})
            per_domain.append(row)
            if number % 50 == 0:
                print(f"LIN {number}/{len(domains)} counts={dict(counts)}", flush=True)
        if args.focus:
            # the first width at which the structure of ONE domain changes inside its certified interval (bisection on decimal widths)
            for prepared, alpha_text, kw, first in domains:
                if first.result.batch.patch_domain_id.value != args.focus:
                    continue
                kw = {**kw, "digests": False}
                base = real_step(prepared, alpha_text, **kw, parametric=True)
                a0 = Fraction(alpha_text)
                low, high = a0, Fraction(args.focus_high)
                for _ in range(40):
                    middle = Fraction(theorem.decimal_text((low + high) / 2))
                    if middle in (low, high):
                        break
                    other = real_step(prepared, theorem.decimal_text(middle), **kw, parametric=True)
                    if other.result.is_materialized and not base.result.structure.differing(other.result.structure):
                        low = middle
                    else:
                        high = middle
                after = real_step(prepared, theorem.decimal_text(high), **kw, parametric=True)
                before = real_step(prepared, theorem.decimal_text(low), **kw, parametric=True)
                report["focus"] = {
                    "patch": args.focus, "alpha0": float(a0), "last_same": float(low), "first_changed": float(high), "share_of_alpha0": float((high - a0) / a0),
                    "interval": base.result.interval.as_record(), "parts": list(before.result.structure.differing(after.result.structure)),
                    "faces_before": len(before.result.batch.faces), "faces_after": len(after.result.batch.faces) if after.result.is_materialized else None,
                    "vertices_before": len(before.result.batch.vertices), "vertices_after": len(after.result.batch.vertices) if after.result.is_materialized else None,
                    "diagnostics_before": sorted(str(x) for x in getattr(before.result.batch, "diagnostics", ()))[:20],
                    "diagnostics_after": sorted(str(x) for x in getattr(after.result.batch, "diagnostics", ()))[:20] if after.result.is_materialized else None,
                    "markers_after": list(after.result.parametric.markers) if after.result.parametric is not None else None,
                    "interval_after": after.result.interval.as_record() if after.result.interval is not None else None,
                }
                print("LIN focus", json.dumps(report["focus"], default=str), flush=True)
        median = lambda v: sorted(v)[len(v) // 2] if v else 0  # noqa: E731
        report.update(
            counts=dict(counts), structure_part_changes=dict(part_changes), theorem_failures=failures[:40], theorem_failure_count=len(failures),
            float_worst_position_m=float_worst[0], float_worst_uv=float_worst[1],
            step_seconds_with_certificate=time_with, step_seconds_without_certificate=time_without,
            events={f"{int(r * 1000) / 10}%": {"per_domain_up_median": median(events_up[r]), "per_domain_up_max": max(events_up[r], default=0), "domains_with_event_up": sum(1 for x in events_up[r] if x),
                                             "per_domain_down_median": median(events_down[r]), "domains_with_event_down": sum(1 for x in events_down[r] if x),
                                             "distinct_event_widths_up": len(global_up[r]), "distinct_event_widths_down": len(global_down[r]), "domains": len(events_up[r])} for r in RANGES},
            per_domain=per_domain,
        )
        print("LIN", json.dumps({k: report[k] for k in ("counts", "structure_part_changes", "theorem_failure_count", "float_worst_position_m", "float_worst_uv", "step_seconds_with_certificate", "step_seconds_without_certificate")}, default=str), flush=True)
        print("LIN events", json.dumps(report["events"]), flush=True)
    except Exception:  # noqa: BLE001
        report["failure"] = traceback.format_exc()
        print(report["failure"], flush=True)
    Path(args.out).write_text(json.dumps(report, indent=1, default=str, sort_keys=True), encoding="utf-8")
    print("LIN_OK" if report["failure"] is None else "LIN_FAILED", flush=True)
    return 0 if report["failure"] is None else 1


if __name__ == "__main__":
    code = main()
    if code:
        raise SystemExit(code)
