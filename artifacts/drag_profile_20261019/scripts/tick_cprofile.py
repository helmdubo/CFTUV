"""Where the CPU of an exact width tick goes, by package (headless; serial in-process path, workers=0; the scene is never saved).

    blender -b <scene> --python-exit-code 1 --python tick_cprofile.py -- --root <tree> --native-path <site> --case building|cover008 --out out.json

Cold press (not profiled), then `--ticks` lean refreshes (WIDTH_REFRESH_LEAN_V1, run_production(lean=True)) at +0.5 % steps under cProfile. tottime is summed per package
(sympy, mpmath, fractions, kernel cftuv_envelope by subpackage, host cftuv, the native extension calls, builtins) and the top functions are listed. Answers: is sympy / PLAN_COMPILE on the tick path.
"""
from __future__ import annotations

import argparse
import cProfile
import io
import json
import pstats
import sys
import time
import traceback
from collections import defaultdict
from pathlib import Path

sys.dont_write_bytecode = True
CASES = {"building": ("testscene", "building", 0.2239, "2", 42, -1.0), "cover008": ("cover008", "cover.008", 0.04214220866560936, "2", 20, 0.004999999888241291)}


def bucket(filename: str, function: str) -> str:
    f = filename.replace("\\", "/")
    if "/sympy/" in f:
        return "sympy"
    if "/mpmath/" in f:
        return "mpmath"
    if f.endswith("/fractions.py"):
        return "fractions"
    if "/decimal" in f or f.endswith("_pydecimal.py"):
        return "decimal"
    if "/cftuv_envelope/" in f:
        rest = f.split("/cftuv_envelope/", 1)[1]
        return "kernel." + (rest.split("/", 1)[0] if "/" in rest else rest)
    if "/cftuv_native/" in f:
        return "native.shim"
    if "/cftuv/" in f:
        return "host"
    if f == "~":
        return "builtin:" + ("native_ext" if "cftuv_native" in function or "_core" in function else "other")
    return "other"


def main() -> int:
    import bpy

    argv = sys.argv[sys.argv.index("--") + 1:]
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", required=True)
    parser.add_argument("--native-path", required=True)
    parser.add_argument("--case", required=True)
    parser.add_argument("--workers", type=int, default=0)
    parser.add_argument("--ticks", type=int, default=3)
    parser.add_argument("--out", required=True)
    args = parser.parse_args(argv)
    root = Path(args.root).resolve()
    sys.path.insert(0, str(root / "tools"))
    import blender_tree_ab as T

    T.load_tree(root, args.native_path)
    report: dict = {"case": args.case, "failure": None}
    try:
        import bmesh
        import cftuv.envelope_production_export as export_module
        from cftuv.analysis import build_analysis_bundle
        from cftuv.analysis_surface import source_revision_from_bmesh
        from cftuv.envelope_debug_session import WINDOW_MANAGER_SESSION_ATTRIBUTE, EnvelopeDebugSessionController
        from cftuv.envelope_gc_pause import gc_paused
        from cftuv.envelope_kernel_backend import kernel_backend_of
        from cftuv.envelope_request_policy import envelope_dissolve_uv_slide, envelope_stretch_budget

        report["native"] = T.verify_native(args.native_path, "")
        scene_kind, mesh, alpha, density, stretch, offset = CASES[args.case]
        scene, obj, settings, mesh_settings = T._open_case(argparse.Namespace(scene_kind=scene_kind, mesh=mesh))
        mesh_settings.kernel_backend = "NATIVE"
        controller = EnvelopeDebugSessionController()
        setattr(bpy.context.window_manager, WINDOW_MANAGER_SESSION_ATTRIBUTE, controller)
        selected = T.select_edges(obj, "stored" if scene_kind == "cover008" else "seam")
        source_bm = bmesh.from_edit_mesh(obj.data)
        source_bm.faces.ensure_lookup_table()
        face_indices = tuple(face.index for face in source_bm.faces)
        revision = source_revision_from_bmesh(source_bm, obj, face_indices)
        key, data_key = int(obj.as_pointer()), int(obj.data.as_pointer())
        bundle = controller.get_analysis_bundle(key, data_key, revision, lambda: build_analysis_bundle(source_bm, face_indices, obj))

        def press(width, lean):
            with gc_paused():
                return export_module.run_production(
                    controller, bundle, frozenset(selected), width, source_object_key=key, source_data_key=data_key, density=density,
                    developable_stretch_budget=envelope_stretch_budget(stretch), silhouette_uv_slide=envelope_dissolve_uv_slide(settings.envelope_debug_dissolve_uv_tolerance),
                    workers=0, kernel_backend=kernel_backend_of(mesh_settings), lean=lean,
                )

        t0 = time.perf_counter()
        press(alpha, False)
        report["cold_seconds_w0"] = time.perf_counter() - t0
        import sympy

        calls = defaultdict(int)
        profile = cProfile.Profile()
        walls = []
        for k in range(1, args.ticks + 1):
            t0 = time.perf_counter()
            profile.enable()
            press(alpha * (1 + 0.005 * k), True)
            profile.disable()
            walls.append(time.perf_counter() - t0)
        report["tick_walls_w0_profiled"] = walls
        stats = pstats.Stats(profile)
        per = defaultdict(float)
        total = 0.0
        sympy_entry = defaultdict(float)
        for (filename, line, function), (cc, nc, tt, ct, callers) in stats.stats.items():
            b = bucket(filename, function)
            per[b] += tt
            total += tt
            if b == "sympy":
                for (cf, cl, cfun), _ in callers.items():
                    if bucket(cf, cfun) != "sympy":
                        sympy_entry[f"{Path(cf).name}:{cfun}"] += ct if False else 0.0
        report["tottime_total"] = total
        report["tottime_by_bucket"] = dict(sorted(per.items(), key=lambda x: -x[1]))
        # who calls into sympy from outside (cumulative of the sympy functions called directly by non-sympy code)
        entry = defaultdict(float)
        for (filename, line, function), (cc, nc, tt, ct, callers) in stats.stats.items():
            if bucket(filename, function) != "sympy":
                continue
            for (cf, cl, cfun), (ccc, cnc, ctt, cct) in callers.items():
                if bucket(cf, cfun) != "sympy":
                    entry[f"{cf.replace(chr(92), '/').split('/cftuv_envelope/')[-1]}:{cfun} -> {function}"] += cct
        report["sympy_entries_cumulative"] = dict(sorted(entry.items(), key=lambda x: -x[1])[:25])
        out = io.StringIO()
        pstats.Stats(profile, stream=out).sort_stats("cumulative").print_stats(45)
        report["top_cumulative"] = out.getvalue()
        out = io.StringIO()
        pstats.Stats(profile, stream=out).sort_stats("tottime").print_stats(30)
        report["top_tottime"] = out.getvalue()
        print("TICKPROF", json.dumps({"walls": walls, "by_bucket": {k: round(v, 3) for k, v in report["tottime_by_bucket"].items()}}), flush=True)
    except Exception:  # noqa: BLE001
        report["failure"] = traceback.format_exc()
        print(report["failure"], flush=True)
    Path(args.out).write_text(json.dumps(report, indent=1, default=str), encoding="utf-8")
    print("TICKPROF_OK" if report["failure"] is None else "TICKPROF_FAILED", flush=True)
    return 0 if report["failure"] is None else 1


if __name__ == "__main__":
    code = main()
    if code:
        raise SystemExit(code)
