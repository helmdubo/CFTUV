"""blender_drag_accuracy: the preview model of a tree against the exact result, tick by tick, and what it costs (S6a; headless Blender only, the scene is never saved).

    blender -b <scene.blend> --python-exit-code 1 --python tools/blender_drag_accuracy.py -- --root <tree under test> --native-path <dir> --native-prefix <p> \\
        --scene testscene|cover008 --mesh M --alpha A --density 2 --stretch 42 [--offset O] --workers 12 --steps 10 --rel 0.005 --out out.json

The script is run from the BRANCH checkout against either tree (`--root`): the helpers of `blender_tree_ab` are the branch's own.

One tree per run (base = quadratic model with primes, branch = affine certificate). Cold exact run at the base width; the model of the tree is built the way the product builds it:
  * a tree with primes: `next_prime_alpha` + a lean exact run + `finish_live_run(prime=True)` until the model is quadratic (the tool entry, `request_prime`), time measured;
  * a tree with the affine certificate: `build_model(sample)` (the tool entry, `ensure_model`), time measured.
Then, for the patterns `up` (+rel .. +steps*rel) and `down`, per tick: the model frame (`positions_and_uvs`, float64) against the EXACT run at that width (lean run, commit-grade geometry): per live domain
the largest vertex / UV error when the domain keeps its structure (token and structure digest of the base), live domains whose structure switched are counted apart (the next exact run refutes them),
held domains by reason. The model is never re-based. Then the per-event frame cost (`evaluate`, p50 / p95 / max over repeated calls).
"""

from __future__ import annotations

import argparse
import inspect
import json
import os
import statistics
import sys
import time
import traceback
from pathlib import Path
from types import SimpleNamespace

sys.dont_write_bytecode = True
# SNAPSHOT_BYTE_CHECK_SAMPLED_V1: the gates and the measurements of the tools check every domain whole (the product button samples). It costs the cold press only.
os.environ["CFTUV_SNAPSHOT_BYTE_CHECK"] = "all"
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
    parser.add_argument("--safety", type=float, default=None, help="override DISPLAY_APPROXIMATE_ERROR_SAFETY (0 = the display-approximate domains live to the flat reach ceiling: the whole sweep is measured)")
    parser.add_argument("--out", required=True)
    return parser.parse_args(sys.argv[sys.argv.index("--") + 1 :] if "--" in sys.argv else [])


def percentile(values, fraction):
    if not values:
        return None
    ordered = sorted(values)
    return ordered[min(len(ordered) - 1, int(round(fraction * (len(ordered) - 1))))]


class Session:
    """The scene, the controller, the analysis bundle and the exact runs of one probe."""

    def __init__(self, args, tree) -> None:
        import bmesh
        import bpy

        from cftuv.analysis import build_analysis_bundle
        from cftuv.analysis_surface import source_revision_from_bmesh
        from cftuv.envelope_debug_session import WINDOW_MANAGER_SESSION_ATTRIBUTE, EnvelopeDebugSessionController
        from cftuv.envelope_kernel_backend import kernel_backend_of

        self.args, self.tree = args, tree
        _scene, obj, self.settings, self.mesh_settings = tree._open_case(SimpleNamespace(scene_kind=args.scene, mesh=args.mesh))
        self.mesh_settings.kernel_backend = "NATIVE"
        if args.offset >= 0:
            self.mesh_settings.offset = args.offset
        self.settings.envelope_debug_fan_density = args.density
        self.settings.envelope_debug_max_stretch = args.stretch
        self.settings.envelope_debug_workers = args.workers
        controller = getattr(bpy.context.window_manager, WINDOW_MANAGER_SESSION_ATTRIBUTE, None)
        if not isinstance(controller, EnvelopeDebugSessionController):
            controller = EnvelopeDebugSessionController()
            setattr(bpy.context.window_manager, WINDOW_MANAGER_SESSION_ATTRIBUTE, controller)
        self.controller = controller
        tree.prepare_pool(SimpleNamespace(origin_check="off", out=args.out, workers=args.workers), {}, {})
        self.selected = tree.select_edges(obj, "stored" if args.scene == "cover008" else "seam")
        bm = bmesh.from_edit_mesh(obj.data)
        bm.faces.ensure_lookup_table()
        indices = tuple(face.index for face in bm.faces)
        revision = source_revision_from_bmesh(bm, obj, indices)
        self.key, self.data_key = int(obj.as_pointer()), int(obj.data.as_pointer())
        self.bundle = controller.get_analysis_bundle(self.key, self.data_key, revision, lambda: build_analysis_bundle(bm, indices, obj))
        self.offset = float(self.mesh_settings.offset)
        self.backend = kernel_backend_of(self.mesh_settings)

    def exact(self, alpha, *, lean):
        import cftuv.envelope_production_export as export_module
        import cftuv.envelope_production_mesh as mesh_module
        from cftuv.envelope_gc_pause import gc_paused
        from cftuv.envelope_request_policy import envelope_dissolve_uv_slide, envelope_stretch_budget

        args, lean = self.args, lean and "lean" in inspect.signature(export_module.run_production).parameters
        with gc_paused():
            run = export_module.run_production(
                self.controller, self.bundle, frozenset(self.selected), alpha, source_object_key=self.key, source_data_key=self.data_key, density=args.density,
                developable_stretch_budget=envelope_stretch_budget(args.stretch), silhouette_uv_slide=envelope_dissolve_uv_slide(self.settings.envelope_debug_dissolve_uv_tolerance),
                workers=args.workers if args.workers >= 2 else 0, kernel_backend=self.backend, **({"lean": True} if lean else {}),
            )
            arrays = mesh_module.build_mesh_arrays(run.results, self.offset, **({"reports": False} if lean else {}))
        return run, arrays


def build_tool_entry_model(session, base):
    """`(model, prime runs)`: the model of the tree the way its tool entry builds it."""

    import cftuv.envelope_width_mesh_preview as preview
    import cftuv.envelope_width_preview_model as pm

    if "prime" not in inspect.signature(preview.finish_live_run).parameters:
        return pm.build_model(base), 0
    aux, model, runs = (), None, 0
    for _attempt in range(preview.PRIME_ATTEMPTS):
        alpha = preview.next_prime_alpha(base, aux)
        if alpha is None:
            break
        run, _arrays = session.exact(alpha, lean=True)
        live = preview.finish_live_run(run, alpha=alpha, offset=session.offset, key=KEY, displayed=base, aux=aux, previous=None, prime=True, trust=None, lean=True)
        aux, model, runs = (live.sample, *aux), live.model, runs + 1
    return model, runs


def judge_tick(model, base, sample, alpha):
    """One tick: live / held / structure-switched domains and the largest errors of the live domains that kept their structure."""

    import numpy as np

    import cftuv.envelope_width_preview_model as pm

    positions, uvs, live = pm.positions_and_uvs(model, alpha)
    twins = {(item.patch_id, item.domain_id): item for item in sample.domains}
    # The tree under test may not know display-approximate domains (the base tree): no such attribute, no such class.
    near = getattr(model, "approximate", None)
    held_reason = {}
    if not live.all():
        # `evaluate` names every held domain, in the order of the non-live domains (reach end, interval end, a reason of the build).
        frame = pm.evaluate(model, alpha)
        held_reason = {int(index): reason for index, (_patch, reason) in zip(np.flatnonzero(~live), frame.held)}
    errors_position, errors_uv, switched, held, reasons = [], [], 0, 0, {}
    exact_position, near_position, near_uv, near_domains = [], [], [], []
    for index, domain in enumerate(base.domains):
        if not live[index]:
            held += 1
            row = model.rows[index]
            served = getattr(pm, "SERVED_OUTCOMES", (pm.DOMAIN_MODELLED,))
            reason = held_reason.get(index) if row[2] in served else row[2]
            reason = reason or "OUTSIDE_INTERVAL_OR_REACH"
            reasons[reason] = reasons.get(reason, 0) + 1
            continue
        twin = twins.get((domain.patch_id, domain.domain_id))
        if twin is None or twin.token != domain.token or twin.structure != domain.structure or twin.vertices.shape != domain.vertices.shape or twin.loop_count != domain.loop_count:
            switched += 1
            continue
        position_error = float(np.max(np.abs(positions[domain.vertices] - sample.positions[twin.vertices]), initial=0.0))
        mine = uvs[domain.loop_start : domain.loop_start + domain.loop_count]
        exact = sample.uvs[twin.loop_start : twin.loop_start + twin.loop_count]
        uv_error = float(np.max(np.abs(mine - exact), initial=0.0))
        errors_position.append(position_error)
        errors_uv.append(uv_error)
        if near is not None and near[index]:
            near_position.append(position_error)
            near_uv.append(uv_error)
            near_domains.append(
                {
                    "patch": domain.patch_id, "max_position_m": position_error, "max_uv": uv_error, "rate": getattr(domain, "approximate_rate", None),
                    "reach": model.rows[index][4], "offset": getattr(base, "offset", None), "dt": alpha - base.alpha,
                }
            )
        else:
            exact_position.append(position_error)
    return {
        "domains": len(base.domains), "live": int(live.sum()), "held": held, "live_switched": switched, "judged": len(errors_position),
        "max_position_m": max(errors_position, default=0.0), "max_uv": max(errors_uv, default=0.0),
        "p50_position_m": percentile(errors_position, 0.5), "p99_position_m": percentile(errors_position, 0.99), "held_reasons": reasons,
        # Exact affine domains and display-approximate ones (PREVIEW_DEVELOPABLE_FROZEN_NORMAL_V1) apart: the second class is bounded by a limit, the first is rounding.
        "exact_judged": len(exact_position), "exact_max_position_m": max(exact_position, default=0.0),
        "approx_judged": len(near_position), "approx_max_position_m": max(near_position, default=0.0), "approx_max_uv": max(near_uv, default=0.0), "approx_domains": near_domains,
    }


def run_ticks(session, model, base):
    import cftuv.envelope_width_mesh_preview as preview

    args, ticks = session.args, []
    for pattern in ("up", "down"):
        for number in range(1, args.steps + 1):
            rel = args.rel * number if pattern == "up" else -args.rel * number
            alpha = float(str(round(args.alpha * (1.0 + rel), 12)))
            run, arrays = session.exact(alpha, lean=True)
            record = judge_tick(model, base, preview.build_sample(run.results, arrays, KEY, alpha), alpha)
            ticks.append({"pattern": pattern, "rel": rel, "alpha": alpha, **record})
            print(
                f"ACC {pattern} {rel:+.3f} live={record['live']}/{record['domains']} switched={record['live_switched']} judged={record['judged']} "
                f"max_pos={record['max_position_m']:.3e} max_uv={record['max_uv']:.3e} exact_pos={record['exact_max_position_m']:.3e} "
                f"approx({record['approx_judged']})_pos={record['approx_max_position_m']:.3e}",
                flush=True,
            )
    return ticks


def frame_cost(model, ticks, repeats):
    import cftuv.envelope_width_preview_model as pm

    milliseconds = []
    for _ in range(repeats):
        for tick in ticks:
            begun = time.perf_counter()
            pm.evaluate(model, tick["alpha"])
            milliseconds.append((time.perf_counter() - begun) * 1000.0)
    return {"p50": statistics.median(milliseconds), "p95": percentile(milliseconds, 0.95), "max": max(milliseconds), "n": len(milliseconds)}


def probe(args, tree, report) -> None:
    import cftuv.envelope_width_mesh_preview as preview
    import cftuv.envelope_width_preview_model as pm

    if args.safety is not None and hasattr(pm, "DISPLAY_APPROXIMATE_ERROR_SAFETY"):
        pm.DISPLAY_APPROXIMATE_ERROR_SAFETY = args.safety
    report["safety"] = getattr(pm, "DISPLAY_APPROXIMATE_ERROR_SAFETY", None)
    session = Session(args, tree)
    started = time.perf_counter()
    run, arrays = session.exact(args.alpha, lean=False)
    report["cold_seconds"] = time.perf_counter() - started
    base = preview.build_sample(run.results, arrays, KEY, args.alpha)
    report["domains"] = len(base.domains)
    import gc

    collections_before = [item["collections"] for item in gc.get_stats()]
    entered = time.perf_counter()
    model, prime_runs = build_tool_entry_model(session, base)
    report["tool_entry_seconds"], report["prime_runs"], report["model_class"] = time.perf_counter() - entered, prime_runs, type(model).__name__
    # Where a slow entry comes from: collections of the cyclic collector inside the call (a generation-2 pass over the heap of a big scene costs hundreds of ms) and the steady-state cost.
    report["tool_entry_gc_collections"] = [after["collections"] - before for before, after in zip(collections_before, gc.get_stats())]
    repeats = []
    for _ in range(3):
        begun = time.perf_counter()
        pm.build_model(base)
        repeats.append(time.perf_counter() - begun)
    report["tool_entry_repeat_seconds"] = repeats
    if not isinstance(model, pm.PreviewModelV1):
        raise RuntimeError(f"no model: {getattr(model, 'outcome', model)}")
    report["model"] = {
        "modelled": model.modelled_domains, "approximate": getattr(model, "approximate_domains", 0), "domains": model.domain_count, "reasons": model.reason_counts(), "own_bytes": model.own_bytes,
    }
    report["ticks"] = run_ticks(session, model, base)
    report["frame_ms"] = frame_cost(model, report["ticks"], args.repeats)
    print("ACC frame ms", report["frame_ms"], "tool entry s", round(report["tool_entry_seconds"], 3), "prime runs", prime_runs, flush=True)


def main() -> int:
    import bpy

    args = _arguments()
    root = Path(args.root).resolve()
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    import blender_tree_ab as tree

    tree.load_tree(root, args.native_path)
    import mpmath  # noqa: F401  (Blender's profile carries them; imported early so a missing one fails here)
    import sympy  # noqa: F401
    import cftuv_envelope.materialize.step  # noqa: F401

    report: dict = {"root": str(root), "mesh": args.mesh, "scene_kind": args.scene, "alpha": args.alpha, "workers": args.workers, "blender": bpy.app.version_string, "failure": None}
    failure = None
    try:
        report["native"] = tree.verify_native(args.native_path, args.native_prefix)
        probe(args, tree, report)
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
