"""S6a drag accuracy: the preview model of a tree against the exact result, tick by tick, and what it costs (headless Blender only; the scene is never saved).

blender -b <scene.blend> --python-exit-code 1 --python drag_accuracy.py -- --tools <branch tools dir> --root <tree under test> --native-path <dir> --native-prefix <p> \
    --scene testscene|cover008 --mesh M --alpha A --density 2 --stretch 42 [--offset O] --workers 12 --steps 10 --rel 0.005 --out out.json

One tree per run (base = quadratic model with primes, branch = affine certificate). Cold exact run at the base width; the model of the tree is built the way the product builds it:
  * base tree: `next_prime_alpha` + a lean exact run + `finish_live_run(prime=True)` until the model is quadratic (the tool entry, `request_prime`), time measured;
  * branch tree: `build_model(sample)` (the tool entry, `ensure_model`), time measured.
Then, for the patterns `up` (+rel .. +steps*rel) and `down`, per tick: the model frame (`positions_and_uvs`, float64) against the EXACT run at that width (lean run, commit-grade geometry): per live domain
the largest vertex / UV error when the domain keeps its structure (token and structure digest of the base), live domains whose structure switched are counted apart (the next exact run refutes them),
held domains by reason. The model is never re-based. Then the per-event frame cost (`evaluate`, p50 / p95 / max over repeated calls).
"""

from __future__ import annotations

import argparse
import inspect
import json
import statistics
import sys
import time
import traceback
from pathlib import Path
from types import SimpleNamespace

sys.dont_write_bytecode = True
KEY = ("probe",)


def _arguments():
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", required=True)
    parser.add_argument("--native-path", required=True)
    parser.add_argument("--native-prefix", default="")
    parser.add_argument("--scene", default="testscene")
    parser.add_argument("--mesh", required=True)
    parser.add_argument("--alpha", type=float, required=True)
    parser.add_argument("--density", default="2")
    parser.add_argument("--stretch", type=int, default=42)
    parser.add_argument("--offset", type=float, default=-1.0)
    parser.add_argument("--workers", type=int, default=12)
    parser.add_argument("--steps", type=int, default=10)
    parser.add_argument("--rel", type=float, default=0.005)
    parser.add_argument("--repeats", type=int, default=20)
    parser.add_argument("--out", required=True)
    return parser.parse_args(sys.argv[sys.argv.index("--") + 1 :] if "--" in sys.argv else [])


def percentile(values, fraction):
    if not values:
        return None
    ordered = sorted(values)
    return ordered[min(len(ordered) - 1, int(round(fraction * (len(ordered) - 1))))]


def main() -> int:
    import bpy

    args = _arguments()
    root = Path(args.root).resolve()
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    import blender_tree_ab as T
    import numpy as np

    T.load_tree(root, args.native_path)
    import mpmath  # noqa: F401  (Blender's profile carries them; imported early so a missing one fails here)
    import sympy  # noqa: F401
    import bmesh
    import cftuv.envelope_production_export as export_module
    import cftuv.envelope_production_mesh as mesh_module
    import cftuv.envelope_width_mesh_preview as preview
    import cftuv.envelope_width_preview_model as pm
    import cftuv_envelope.materialize.step  # noqa: F401  (the kernel stage is imported before the add-on uses it)
    from cftuv.analysis import build_analysis_bundle
    from cftuv.analysis_surface import source_revision_from_bmesh
    from cftuv.envelope_debug_session import WINDOW_MANAGER_SESSION_ATTRIBUTE, EnvelopeDebugSessionController
    from cftuv.envelope_gc_pause import gc_paused
    from cftuv.envelope_kernel_backend import kernel_backend_of
    from cftuv.envelope_request_policy import envelope_dissolve_uv_slide, envelope_stretch_budget

    report: dict = {"root": str(root), "mesh": args.mesh, "scene_kind": args.scene, "alpha": args.alpha, "workers": args.workers, "blender": bpy.app.version_string, "failure": None}
    failure = None
    try:
        report["native"] = T.verify_native(args.native_path, args.native_prefix)
        ns = SimpleNamespace(scene_kind=args.scene, mesh=args.mesh)
        scene, obj, settings, mesh_settings = T._open_case(ns)
        mesh_settings.kernel_backend = "NATIVE"
        if args.offset >= 0:
            mesh_settings.offset = args.offset
        settings.envelope_debug_fan_density = args.density
        settings.envelope_debug_max_stretch = args.stretch
        settings.envelope_debug_workers = args.workers
        controller = getattr(bpy.context.window_manager, WINDOW_MANAGER_SESSION_ATTRIBUTE, None)
        if not isinstance(controller, EnvelopeDebugSessionController):
            controller = EnvelopeDebugSessionController()
            setattr(bpy.context.window_manager, WINDOW_MANAGER_SESSION_ATTRIBUTE, controller)
        T.prepare_pool(SimpleNamespace(origin_check="off", out=args.out, workers=args.workers), report, {})
        selected = T.select_edges(obj, "stored" if args.scene == "cover008" else "seam")
        source_bm = bmesh.from_edit_mesh(obj.data)
        source_bm.faces.ensure_lookup_table()
        face_indices = tuple(face.index for face in source_bm.faces)
        revision = source_revision_from_bmesh(source_bm, obj, face_indices)
        key, data_key = int(obj.as_pointer()), int(obj.data.as_pointer())
        bundle = controller.get_analysis_bundle(key, data_key, revision, lambda: build_analysis_bundle(source_bm, face_indices, obj))
        offset = float(mesh_settings.offset)
        lean_supported = "lean" in inspect.signature(export_module.run_production).parameters

        def exact(alpha, *, lean):
            with gc_paused():
                run = export_module.run_production(
                    controller, bundle, frozenset(selected), alpha, source_object_key=key, source_data_key=data_key, density=args.density,
                    developable_stretch_budget=envelope_stretch_budget(args.stretch), silhouette_uv_slide=envelope_dissolve_uv_slide(settings.envelope_debug_dissolve_uv_tolerance),
                    workers=args.workers if args.workers >= 2 else 0, kernel_backend=kernel_backend_of(mesh_settings), **({"lean": True} if lean and lean_supported else {}),
                )
                arrays = mesh_module.build_mesh_arrays(run.results, offset, **({"reports": False} if lean and lean_supported else {}))
            return run, arrays

        started = time.perf_counter()
        run, arrays = exact(args.alpha, lean=False)
        report["cold_seconds"] = time.perf_counter() - started
        base = preview.build_sample(run.results, arrays, KEY, args.alpha)
        report["domains"] = len(base.domains)
        # --- the tool entry: the model of the tree, built the way the product builds it
        live_params = inspect.signature(preview.finish_live_run).parameters
        primes = "prime" in live_params
        entry_started = time.perf_counter()
        prime_runs = 0
        if primes:
            aux = ()
            model = None
            for _attempt in range(preview.PRIME_ATTEMPTS):
                alpha = preview.next_prime_alpha(base, aux)
                if alpha is None:
                    break
                prime_run, _prime_arrays = exact(alpha, lean=True)
                live = preview.finish_live_run(prime_run, alpha=alpha, offset=offset, key=KEY, displayed=base, aux=aux, previous=None, prime=True, trust=None, lean=True)
                aux = (live.sample, *aux)
                model = live.model
                prime_runs += 1
        else:
            model = pm.build_model(base)
        report["tool_entry_seconds"] = time.perf_counter() - entry_started
        report["prime_runs"] = prime_runs
        report["model_class"] = type(model).__name__
        if not isinstance(model, pm.PreviewModelV1):
            raise RuntimeError(f"no model: {getattr(model, 'outcome', model)}")
        report["model"] = {"modelled": model.modelled_domains, "domains": model.domain_count, "reasons": model.reason_counts(), "own_bytes": model.own_bytes}
        ticks = []
        for pattern in ("up", "down"):
            for number in range(1, args.steps + 1):
                rel = args.rel * number if pattern == "up" else -args.rel * number
                alpha = float(str(round(args.alpha * (1.0 + rel), 12)))
                run_k, arrays_k = exact(alpha, lean=True)
                sample = preview.build_sample(run_k.results, arrays_k, KEY, alpha)
                positions, uvs, live = pm.positions_and_uvs(model, alpha)
                twins = {(item.patch_id, item.domain_id): item for item in sample.domains}
                errors_position, errors_uv, switched, held, reasons = [], [], 0, 0, {}
                for index, domain in enumerate(base.domains):
                    if not live[index]:
                        held += 1
                        row = model.rows[index]
                        reason = row[2] if row[2] != pm.DOMAIN_MODELLED else ("OUTSIDE_INTERVAL_OR_REACH")
                        reasons[reason] = reasons.get(reason, 0) + 1
                        continue
                    twin = twins.get((domain.patch_id, domain.domain_id))
                    if twin is None or twin.token != domain.token or twin.structure != domain.structure or twin.vertices.shape != domain.vertices.shape or twin.loop_count != domain.loop_count:
                        switched += 1
                        continue
                    errors_position.append(float(np.max(np.abs(positions[domain.vertices] - sample.positions[twin.vertices]), initial=0.0)))
                    errors_uv.append(
                        float(
                            np.max(
                                np.abs(
                                    uvs[domain.loop_start : domain.loop_start + domain.loop_count] - sample.uvs[twin.loop_start : twin.loop_start + twin.loop_count]
                                ),
                                initial=0.0,
                            )
                        )
                    )
                ticks.append(
                    {
                        "pattern": pattern, "rel": rel, "alpha": alpha, "domains": len(base.domains), "live": int(live.sum()), "held": held, "live_switched": switched,
                        "judged": len(errors_position), "max_position_m": max(errors_position, default=0.0), "max_uv": max(errors_uv, default=0.0),
                        "p50_position_m": percentile(errors_position, 0.5), "p99_position_m": percentile(errors_position, 0.99), "held_reasons": reasons,
                    }
                )
                print(f"ACC {pattern} {rel:+.3f} live={int(live.sum())}/{len(base.domains)} switched={switched} judged={len(errors_position)} max_pos={max(errors_position, default=0.0):.3e} max_uv={max(errors_uv, default=0.0):.3e}", flush=True)
        report["ticks"] = ticks
        # --- per-event frame cost
        widths = [ticks[i]["alpha"] for i in range(len(ticks))]
        frame_ms = []
        for _ in range(args.repeats):
            for alpha in widths:
                begun = time.perf_counter()
                pm.evaluate(model, alpha)
                frame_ms.append((time.perf_counter() - begun) * 1000.0)
        report["frame_ms"] = {"p50": statistics.median(frame_ms), "p95": percentile(frame_ms, 0.95), "max": max(frame_ms), "n": len(frame_ms)}
        print("ACC frame ms", report["frame_ms"], "tool entry s", round(report["tool_entry_seconds"], 3), "prime runs", prime_runs, flush=True)
    except Exception:  # noqa: BLE001
        failure = traceback.format_exc()
        print(failure)
    try:
        from cftuv.envelope_domain_pool import shutdown_domain_pool

        shutdown_domain_pool()
    except Exception as exc:  # noqa: BLE001
        print("pool shutdown:", type(exc).__name__, exc)
    report["failure"] = failure
    Path(args.out).write_text(json.dumps(report, ensure_ascii=False, indent=1, sort_keys=True, default=str) + "\n", encoding="utf-8")
    print(f"ACC_{'OK' if failure is None else 'FAILED'} mesh={args.mesh}", flush=True)
    return 0 if failure is None else 1


if __name__ == "__main__":
    code = main()
    if code:
        raise SystemExit(code)
