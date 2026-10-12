"""S6a field theorem + overhead probe (headless Blender only; the scene is never saved).

blender -b <scene.blend> --python-exit-code 1 --python field_theorem.py -- --root <tree> --native-path <dir> --native-prefix <p> --scene testscene|cover008 --mesh M --alpha A \
    --density 2 --stretch 42 [--offset O] --shares 0.001,-0.001,... [--limit N] --out out.json

One cold press (workers=0, every domain in this process, GC paused as the button does) with a recording wrapper around `step_domain` that also orders the
certificate. Then, per captured domain:
  * overhead: FAST_HIT step at alpha0 with and without the certificate (min of repeats);
  * theorem: exact certificate at alpha0 and at alpha0 * (1 + share): value + slope * delta == exact run, EXACTLY (SqrtSumV1), when the structure is unchanged;
  * float error of the affine frame against the real batch of the shifted run.
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
    parser.add_argument("--shares", default="0.001,-0.001,0.005,-0.005,0.02,-0.02,0.05,-0.05")
    parser.add_argument("--limit", type=int, default=0)
    parser.add_argument("--repeats", type=int, default=3)
    parser.add_argument("--profile", type=int, default=0)
    parser.add_argument("--out", required=True)
    return parser.parse_args(sys.argv[sys.argv.index("--") + 1 :] if "--" in sys.argv else [])


def main() -> int:
    import bpy

    args = _arguments()
    root = Path(args.root).resolve()
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    sys.path.insert(0, str(root / "kernel" / "tests"))
    import blender_tree_ab as T

    T.load_tree(root, args.native_path)
    import mpmath  # noqa: F401
    import sympy  # noqa: F401
    import bmesh
    import cftuv.envelope_production_export as export_module
    import cftuv_envelope.materialize.step as step_module
    import parametric_theorem as theorem
    from cftuv.analysis import build_analysis_bundle
    from cftuv.analysis_surface import source_revision_from_bmesh
    from cftuv.envelope_debug_session import WINDOW_MANAGER_SESSION_ATTRIBUTE, EnvelopeDebugSessionController
    from cftuv.envelope_gc_pause import gc_paused
    from cftuv.envelope_kernel_backend import kernel_backend_of
    from cftuv.envelope_request_policy import envelope_dissolve_uv_slide, envelope_stretch_budget

    report: dict = {"root": str(root), "mesh": args.mesh, "scene_kind": args.scene, "alpha": args.alpha, "blender": bpy.app.version_string, "blend": bpy.data.filepath, "failure": None}
    failure = None
    captured: list = []
    real_step = step_module.step_domain
    try:
        report["native"] = T.verify_native(args.native_path, args.native_prefix)
        ns = argparse.Namespace(scene_kind=args.scene, mesh=args.mesh)
        scene, obj, settings, mesh_settings = T._open_case(ns)
        mesh_settings.kernel_backend = "NATIVE"
        if args.offset >= 0:
            mesh_settings.offset = args.offset
        settings.envelope_debug_fan_density = args.density
        settings.envelope_debug_max_stretch = args.stretch
        controller = getattr(bpy.context.window_manager, WINDOW_MANAGER_SESSION_ATTRIBUTE, None)
        if not isinstance(controller, EnvelopeDebugSessionController):
            controller = EnvelopeDebugSessionController()
            setattr(bpy.context.window_manager, WINDOW_MANAGER_SESSION_ATTRIBUTE, controller)
        selected = T.select_edges(obj, "stored" if args.scene == "cover008" else "seam")
        source_bm = bmesh.from_edit_mesh(obj.data)
        source_bm.faces.ensure_lookup_table()
        face_indices = tuple(face.index for face in source_bm.faces)
        revision = source_revision_from_bmesh(source_bm, obj, face_indices)
        key, data_key = int(obj.as_pointer()), int(obj.data.as_pointer())
        bundle = controller.get_analysis_bundle(key, data_key, revision, lambda: build_analysis_bundle(source_bm, face_indices, obj))

        def recording(prepared, alpha_text, **kw):
            kw = {**kw, "parametric": True, "parametric_exact": False}
            found = real_step(prepared, alpha_text, **kw)
            captured.append((prepared, alpha_text, {k: v for k, v in kw.items() if k not in ("parametric", "parametric_exact")}, found))
            return found

        step_module.step_domain = recording
        started = time.perf_counter()
        with gc_paused():
            run = export_module.run_production(
                controller, bundle, frozenset(selected), args.alpha, source_object_key=key, source_data_key=data_key, density=args.density,
                developable_stretch_budget=envelope_stretch_budget(args.stretch), silhouette_uv_slide=envelope_dissolve_uv_slide(settings.envelope_debug_dissolve_uv_tolerance),
                workers=0, kernel_backend=kernel_backend_of(mesh_settings),
            )
        report["press_seconds"] = time.perf_counter() - started
        step_module.step_domain = real_step
        report["domains"] = len(run.results)
        report["materialized"] = sum(1 for item in run.results if item.is_materialized)
        report["captured"] = len(captured)
        print(f"FIELD press {args.mesh} domains={report['domains']} captured={len(captured)} seconds={report['press_seconds']:.2f}", flush=True)
        if args.profile:
            import cProfile
            import io
            import pstats

            from cftuv_envelope.materialize import parametric as parametric_module

            profile = cProfile.Profile()
            real_certificate = parametric_module.certificate_of

            def profiled(*a, **k):
                profile.enable()
                try:
                    return real_certificate(*a, **k)
                finally:
                    profile.disable()

            import cftuv_envelope.materialize.domain as domain_module

            domain_module.certificate_of = profiled
            for prepared, alpha_text, kw, first in captured:
                if first.result.is_materialized:
                    real_step(prepared, alpha_text, **{**kw, "digests": False}, parametric=True)
            domain_module.certificate_of = real_certificate
            out = io.StringIO()
            stats = pstats.Stats(profile, stream=out).sort_stats("tottime")
            stats.print_stats(45)
            report["profile_tottime"] = out.getvalue()
            out = io.StringIO()
            pstats.Stats(profile, stream=out).sort_stats("cumulative").print_stats(30)
            report["profile_cumulative"] = out.getvalue()
            Path(args.out).with_suffix(".prof.txt").write_text(report["profile_tottime"] + "\n\n" + report["profile_cumulative"], encoding="utf-8")
            print("FIELD profile written", flush=True)
        shares = [float(item) for item in args.shares.split(",") if item]
        markers: Counter = Counter()
        counts: Counter = Counter()
        worst_position = worst_uv = 0.0
        overhead_with = overhead_without = 0.0
        measured = 0
        failures: list = []
        per_domain: list = []
        picked = captured if not args.limit else captured[: args.limit]
        for number, (prepared, alpha_text, kw, first) in enumerate(picked):
            if not first.result.is_materialized:
                continue
            kw = {**kw, "digests": False}
            parametric = first.result.parametric
            if parametric is None:
                markers["PARAM_ABSENT"] += 1
            else:
                for name in parametric.markers:
                    markers[name] += 1
                if not parametric.markers:
                    markers["AFFINE"] += 1
            # overhead on FAST_HIT steps at alpha0
            best_without = best_with = None
            for repeat in range(args.repeats):
                begun = time.perf_counter()
                real_step(prepared, alpha_text, **kw, parametric=False)
                took = time.perf_counter() - begun
                best_without = took if best_without is None else min(best_without, took)
                begun = time.perf_counter()
                real_step(prepared, alpha_text, **kw, parametric=True)
                took = time.perf_counter() - begun
                best_with = took if best_with is None else min(best_with, took)
            overhead_without += best_without
            overhead_with += best_with
            measured += 1
            base = real_step(prepared, alpha_text, **kw, parametric_exact=True)
            base_certificate = base.result.parametric
            if not theorem.usable(base_certificate):
                counts["BASE_NOT_USABLE"] += 1
                continue
            alpha0 = Fraction(alpha_text)
            interval = base.result.interval
            for share in shares:
                text = theorem.decimal_text(alpha0 * (1 + Fraction(share)))
                alpha1 = Fraction(text)
                if alpha1 <= 0:
                    continue
                other = real_step(prepared, text, **kw, parametric_exact=True)
                if not other.result.is_materialized:
                    counts["OTHER_REFUSED"] += 1
                    continue
                inside = interval is not None and interval.contains_exact(alpha1)
                outcome, found = theorem.theorem_differences(base, other, alpha0, alpha1)
                counts[outcome + ("_INSIDE" if inside else "_OUTSIDE")] += 1
                if outcome == theorem.JUDGED:
                    if found:
                        failures.append({"patch": base.result.batch.patch_domain_id.value, "share": share, "inside": inside, "differences": [str(x) for x in found[:5]], "count": len(found)})
                    errors = theorem.float_errors(base, other, alpha0, alpha1)
                    if errors is not None and inside:
                        worst_position = max(worst_position, errors[0])
                        worst_uv = max(worst_uv, errors[1])
                        per_domain.append((errors[0], errors[1]))
        report.update(
            markers=dict(markers), counts=dict(counts), theorem_failures=failures[:40], theorem_failure_count=len(failures), worst_position_error=worst_position, worst_uv_error=worst_uv,
            overhead_seconds_without=overhead_without, overhead_seconds_with=overhead_with, overhead_domains=measured,
            overhead_ratio=(overhead_with / overhead_without if overhead_without else None),
        )
        print("FIELD", json.dumps({k: report[k] for k in ("markers", "counts", "theorem_failure_count", "worst_position_error", "worst_uv_error", "overhead_ratio")}, default=str), flush=True)
    except Exception:  # noqa: BLE001
        failure = traceback.format_exc()
        print(failure)
    report["failure"] = failure
    Path(args.out).write_text(json.dumps(report, ensure_ascii=False, indent=1, sort_keys=True, default=str) + "\n", encoding="utf-8")
    print(f"FIELD_{'OK' if failure is None else 'FAILED'} mesh={args.mesh}", flush=True)
    return 0 if failure is None else 1


if __name__ == "__main__":
    code = main()
    if code:
        raise SystemExit(code)
