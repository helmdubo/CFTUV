"""Drag-tick profile: the live width path of the product (blender_drag_ticks.Session.step = _begin -> thread compute -> _validity -> _apply), timed per refresh,
with per-task stage laps INSIDE every pool worker (draglaps.py patched into the worker bootstrap of THIS process only) and the parent laps of blender_drag_ticks.

    blender -b <scene> --python-exit-code 1 --python dragprof_runner.py -- --root <tree> --native-path <site> --case building|cover008 --workers 12
            --sequence lean,incr,commit --reps 6 --rel 0.005 --out out.json

Per rep, one refresh of each kind of `--sequence` at a new width (base * (1 + rel * k), k = 1, 2, ...):
  lean   = WIDTH_REFRESH_LEAN_V1 (the full exact tick while dragging, the verification tick of the incremental policy),
  incr   = WIDTH_REFRESH_INCREMENTAL_V1 as the tool orders it (the policy decides; the record names the plan),
  commit = the full path (the confirm / a slider step / the button's width step).
After each refresh, outside its time: `view_layer.update()` (the depsgraph evaluation the GUI does on redraw) is timed as host.depsgraph.
Nothing is saved. Diagnostic only: the product code is not touched (laps are wrappers installed in memory).
"""
from __future__ import annotations

import argparse
import json
import os
import statistics
import sys
import time
import traceback
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

PC_EXTRA = (
    ("kernel:reference.compile", "validate_snapshot_request_references", "PC.validate_snapshot_request_references"),
    ("kernel:reference.compile", "validate_reference_geometry_certificates", "PC.validate_reference_geometry_certificates"),
    ("kernel:reference.compile", "resolve_corner_selection", "PC.resolve_corner_selection"),
    ("kernel:reference.common", "station_plans_of", "PC.station_plans_of"),
    ("kernel:reference.evaluation_geometry", "build_evaluation_geometry_binding", "PCE.build_evaluation_geometry_binding"),
    ("kernel:reference.evaluation_geometry", "verify_evaluation_geometry_binding", "PCE.verify_evaluation_geometry_binding"),
    ("kernel:backend", "density_compute", "PCD.backend_density_compute"),
    ("kernel:reference.common", "GeometryContext._unit_g", "PCS.unit_g"),
    ("kernel:reference.common", "GeometryContext._face_side_normal", "PCS.face_side_normal"),
)


def main() -> int:
    argv = sys.argv[sys.argv.index("--") + 1:]
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", required=True)
    parser.add_argument("--native-path", required=True)
    parser.add_argument("--case", required=True)
    parser.add_argument("--workers", type=int, default=12)
    parser.add_argument("--sequence", default="lean,incr,commit")
    parser.add_argument("--reps", type=int, default=6)
    parser.add_argument("--rel", type=float, default=0.005)
    parser.add_argument("--laps", type=int, default=1)
    parser.add_argument("--kernel-backend", default="NATIVE")
    parser.add_argument("--out", required=True)
    args = parser.parse_args(argv)
    out = Path(args.out)
    report: dict = {"argv": argv, "pid": os.getpid(), "failure": None}
    sys.path.insert(0, str(Path(args.root) / "tools"))
    import blender_drag_ticks as dt  # sets CFTUV_SNAPSHOT_BYTE_CHECK=all (the cold press only)

    session = None
    try:
        session_args = argparse.Namespace(root=args.root, native_path=args.native_path, case=args.case, workers=args.workers, kernel_backend=args.kernel_backend)
        session = dt.Session(session_args)
        report["native_build_id"] = session.native["build_id"]
        import bpy

        laps = dt.Laps()
        report["parent_laps_found"] = dt.install_laps(laps)
        # extra parent laps (same Laps object)
        import importlib

        extra = (("cftuv.envelope_production_export", "run_production", "P.run_production"), ("cftuv.envelope_width_live", "_apply", "P.apply"),
                 ("cftuv.envelope_production_export", "_adopt_cold", "P.adopt_cold"), ("cftuv.envelope_production_export", "_collect", "P.collect"))
        for module_name, attribute, lap in extra:
            module = importlib.import_module(module_name)
            original = getattr(module, attribute, None)
            if original is None:
                report.setdefault("parent_laps_missing", []).append(f"{module_name}.{attribute}")
                continue

            def make(function, name):
                def timed(*a, **k):
                    started = time.perf_counter()
                    try:
                        return function(*a, **k)
                    finally:
                        laps.seconds[name] = laps.seconds.get(name, 0.0) + time.perf_counter() - started
                        laps.calls[name] = laps.calls.get(name, 0) + 1
                timed.__wrapped__ = function
                return timed

            setattr(module, attribute, make(original, lap))
        dump_dir = str(out.with_suffix("")) + "_workers"
        if args.workers and args.laps:
            import cftuv.envelope_domain_pool as pool_module

            Path(dump_dir).mkdir(parents=True, exist_ok=True)
            call = dt.tree_ab.WORKER_MAIN_CALL
            assert call in pool_module._BOOTSTRAP
            inject = (f"sys.path.insert(0, {str(HERE)!r})\nimport draglaps as _draglaps\n_draglaps.install_worker(spec['package'], {dump_dir!r}, laps=True)\n"
                      f"for _m, _d, _l in {PC_EXTRA!r}:\n    _draglaps.wrap(spec['package'], _m, _d, _l)\n")
            pool_module._BOOTSTRAP = pool_module._BOOTSTRAP.replace(call, inject + call)
            report["worker_dump_dir"] = dump_dir
        report["cold_press_s"] = session.cold_press()
        report["cold_window"] = None
        if bpy.context.mode != "OBJECT":
            bpy.ops.object.mode_set(mode="OBJECT")
        base = float(session.settings.envelope_debug_alpha)
        report["base_alpha"] = base
        mesh = session.decal_mesh
        report["decal_cold"] = {"vertices": len(mesh.vertices), "polygons": len(mesh.polygons), "loops": len(mesh.loops)}
        kinds = [item.strip() for item in args.sequence.split(",") if item.strip()]
        steps = []
        k = 0
        # warm-up (not counted): one lean tick and one commit, so every worker has seen the preparation of its domains once after the button
        for kind in kinds:
            pass
        for rep in range(args.reps):
            for kind in kinds:
                k += 1
                alpha = base * (1.0 + args.rel * k)
                session.controller.width_drag_active = kind != "commit"
                record = session.step(alpha, lean=kind != "commit", laps=laps, incremental=kind == "incr")
                record["kind"], record["rep"] = kind, rep
                started = time.perf_counter()
                bpy.context.view_layer.update()
                record["host_depsgraph"] = time.perf_counter() - started
                m = session.decal_mesh
                record["decal"] = [len(m.vertices), len(m.polygons), len(m.loops)]
                steps.append(record)
                print(f"DRAGPROF {kind} rep={rep} alpha={alpha:.6f} wall={record['wall']:.3f} thread={record['thread']:.3f} apply={record['apply_main']:.3f} dg={record['host_depsgraph']:.4f} plan={record['plan']}", flush=True)
        session.controller.width_drag_active = False
        report["steps"] = steps
        summary = {}
        for kind in kinds:
            rows = [item for item in steps if item["kind"] == kind]
            summary[kind] = {name: statistics.median(item[name] for item in rows) for name in ("wall", "thread", "begin_main", "apply_main", "run_production", "host_depsgraph")}
            summary[kind]["n"] = len(rows)
        report["summary"] = summary
        time.sleep(1.5)  # the workers flush their task records every 0.5 s
    except Exception:  # noqa: BLE001
        report["failure"] = traceback.format_exc()
        print(report["failure"], flush=True)
    try:
        from cftuv.envelope_domain_pool import shutdown_domain_pool

        shutdown_domain_pool()
    except Exception as exc:  # noqa: BLE001
        report["shutdown"] = repr(exc)
    out.write_text(json.dumps(report, indent=1, default=str, sort_keys=True), encoding="utf-8")
    print("DRAGPROF_OK" if report["failure"] is None else "DRAGPROF_FAILED", flush=True)
    return 0 if report["failure"] is None else 1


if __name__ == "__main__":
    code = main()
    if code:
        raise SystemExit(code)
