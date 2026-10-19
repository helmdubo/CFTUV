"""Headless capture: one cold press (workers=0) of a frozen case; pickles the preparation and step arguments of named domains.

    blender -b <scene> --python-exit-code 1 --python capture_focus.py -- --root D:/CFTUV-lazydrag --native-path D:/lazydrag-scratch/site --case cover008 --patches 1146c49e --out D:/lazydrag-scratch/focus
The scene is never saved.
"""
from __future__ import annotations

import argparse
import json
import pickle
import sys
import time
import traceback
from pathlib import Path

sys.dont_write_bytecode = True
CASES = {"building": ("testscene", "building", 0.2239, "2", 42, -1.0), "cover008": ("cover008", "cover.008", 0.04214220866560936, "2", 20, 0.004999999888241291)}


def main() -> int:
    import bpy

    argv = sys.argv[sys.argv.index("--") + 1:]
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", required=True)
    parser.add_argument("--native-path", required=True)
    parser.add_argument("--case", required=True)
    parser.add_argument("--patches", default="")
    parser.add_argument("--all", action="store_true")
    parser.add_argument("--out", required=True)
    args = parser.parse_args(argv)
    root = Path(args.root).resolve()
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    sys.path.insert(0, str(root / "tools"))
    import blender_tree_ab as T

    T.load_tree(root, args.native_path)
    import bmesh
    import cftuv.envelope_production_export as export_module
    import cftuv_envelope.materialize.step as step_module
    from cftuv.analysis import build_analysis_bundle
    from cftuv.analysis_surface import source_revision_from_bmesh
    from cftuv.envelope_debug_session import WINDOW_MANAGER_SESSION_ATTRIBUTE, EnvelopeDebugSessionController
    from cftuv.envelope_gc_pause import gc_paused
    from cftuv.envelope_kernel_backend import kernel_backend_of
    from cftuv.envelope_request_policy import envelope_dissolve_uv_slide, envelope_stretch_budget

    report = {"case": args.case, "failure": None}
    scene_kind, mesh, alpha, density, stretch, offset = CASES[args.case]
    captured = []
    real_step = step_module.step_domain
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
            captured.append((prepared, alpha_text, kw, found))
            return found

        step_module.step_domain = recording
        started = time.perf_counter()
        with gc_paused():
            run = export_module.run_production(
                controller, bundle, frozenset(selected), alpha, source_object_key=key, source_data_key=data_key, density=density,
                developable_stretch_budget=envelope_stretch_budget(stretch),
                silhouette_uv_slide=envelope_dissolve_uv_slide(settings.envelope_debug_dissolve_uv_tolerance),
                workers=0, kernel_backend=kernel_backend_of(mesh_settings),
            )
        step_module.step_domain = real_step
        report["press_seconds"] = time.perf_counter() - started
        report["captured"] = len(captured)
        wanted = [p for p in args.patches.split(",") if p]
        saved = []
        for number, (prepared, alpha_text, kw, found) in enumerate(captured):
            if not found.result.is_materialized:
                continue
            patch = found.result.batch.patch_domain_id.value
            if not args.all and not any(w in patch for w in wanted):
                continue
            name = f"{number:05d}_{patch.split(':')[-1][:12]}.pkl"
            with open(out / name, "wb") as handle:
                pickle.dump({"prepared": prepared, "alpha_text": alpha_text, "kw": kw, "patch": patch,
                             "interval": None if found.result.interval is None else found.result.interval.as_record()}, handle, protocol=pickle.HIGHEST_PROTOCOL)
            saved.append(name)
        report["saved"] = saved
    except Exception:  # noqa: BLE001
        report["failure"] = traceback.format_exc()
        print(report["failure"], flush=True)
    (out / f"capture_{args.case}.json").write_text(json.dumps(report, indent=1, default=str), encoding="utf-8")
    print("CAP_OK" if report["failure"] is None else "CAP_FAILED", flush=True)
    return 0 if report["failure"] is None else 1


if __name__ == "__main__":
    code = main()
    if code:
        raise SystemExit(code)
