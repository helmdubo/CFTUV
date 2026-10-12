"""blender_affine_probe: the affine drag certificate on the domains of a real scene (S6a; headless Blender only, the scene is never saved).

    blender -b <scene.blend> --python-exit-code 1 --python tools/blender_affine_probe.py -- --root <tree> --native-path <dir> --native-prefix <p> --scene testscene|cover008 --mesh M \\
        --alpha A --density 2 --stretch 42 [--offset O] [--shares 0.001,-0.001,...] [--limit N] [--profile 1] --out out.json

One cold press (workers=0, every domain in this process, GC paused as the button does) with a recording wrapper around `step_domain` that also orders the certificate. Then, per captured domain:
  * overhead: FAST_HIT step at alpha0 with and without the certificate (min of repeats);
  * theorem: exact certificate at alpha0 and at alpha0 * (1 + share): value + slope * delta == exact run, EXACTLY (SqrtSumV1), when the structure is unchanged
    (`kernel/tests/parametric_theorem.py`);
  * float error of the affine frame against the real batch of the shifted run (rounding level).
`--profile 1` adds a cProfile of the certificate over every domain.
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time
import traceback
from collections import Counter
from fractions import Fraction
from pathlib import Path
from types import SimpleNamespace

sys.dont_write_bytecode = True
# SNAPSHOT_BYTE_CHECK_SAMPLED_V1: the gates and the measurements of the tools check every domain whole (the product button samples). It costs the cold press only.
os.environ["CFTUV_SNAPSHOT_BYTE_CHECK"] = "all"


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


def capture_press(args, tree, step_module, real_step):
    """The cold press with every `step_domain` call recorded (and ordering the certificate): `(captured calls, run, seconds)`."""

    import bmesh
    import bpy

    import cftuv.envelope_production_export as export_module
    from cftuv.analysis import build_analysis_bundle
    from cftuv.analysis_surface import source_revision_from_bmesh
    from cftuv.envelope_debug_session import WINDOW_MANAGER_SESSION_ATTRIBUTE, EnvelopeDebugSessionController
    from cftuv.envelope_gc_pause import gc_paused
    from cftuv.envelope_kernel_backend import kernel_backend_of
    from cftuv.envelope_request_policy import envelope_dissolve_uv_slide, envelope_stretch_budget

    _scene, obj, settings, mesh_settings = tree._open_case(SimpleNamespace(scene_kind=args.scene, mesh=args.mesh))
    mesh_settings.kernel_backend = "NATIVE"
    if args.offset >= 0:
        mesh_settings.offset = args.offset
    settings.envelope_debug_fan_density = args.density
    settings.envelope_debug_max_stretch = args.stretch
    controller = getattr(bpy.context.window_manager, WINDOW_MANAGER_SESSION_ATTRIBUTE, None)
    if not isinstance(controller, EnvelopeDebugSessionController):
        controller = EnvelopeDebugSessionController()
        setattr(bpy.context.window_manager, WINDOW_MANAGER_SESSION_ATTRIBUTE, controller)
    selected = tree.select_edges(obj, "stored" if args.scene == "cover008" else "seam")
    bm = bmesh.from_edit_mesh(obj.data)
    bm.faces.ensure_lookup_table()
    indices = tuple(face.index for face in bm.faces)
    revision = source_revision_from_bmesh(bm, obj, indices)
    key, data_key = int(obj.as_pointer()), int(obj.data.as_pointer())
    bundle = controller.get_analysis_bundle(key, data_key, revision, lambda: build_analysis_bundle(bm, indices, obj))
    captured: list = []

    def recording(prepared, alpha_text, **kw):
        kw = {**kw, "parametric": True, "parametric_exact": False}
        found = real_step(prepared, alpha_text, **kw)
        captured.append((prepared, alpha_text, {k: v for k, v in kw.items() if k not in ("parametric", "parametric_exact")}, found))
        return found

    step_module.step_domain = recording
    started = time.perf_counter()
    try:
        with gc_paused():
            run = export_module.run_production(
                controller, bundle, frozenset(selected), args.alpha, source_object_key=key, source_data_key=data_key, density=args.density,
                developable_stretch_budget=envelope_stretch_budget(args.stretch), silhouette_uv_slide=envelope_dissolve_uv_slide(settings.envelope_debug_dissolve_uv_tolerance),
                workers=0, kernel_backend=kernel_backend_of(mesh_settings),
            )
    finally:
        step_module.step_domain = real_step
    return captured, run, time.perf_counter() - started


def profile_certificates(captured, real_step, report, out_path):
    """cProfile of `certificate_of` over every captured domain (the step is FAST_HIT: the certificate is the only new work)."""

    import cProfile
    import io
    import pstats

    import cftuv_envelope.materialize.domain as domain_module

    profile = cProfile.Profile()
    real = domain_module.certificate_of

    def profiled(*a, **k):
        profile.enable()
        try:
            return real(*a, **k)
        finally:
            profile.disable()

    domain_module.certificate_of = profiled
    try:
        for prepared, alpha_text, kw, first in captured:
            if first.result.is_materialized:
                real_step(prepared, alpha_text, **{**kw, "digests": False}, parametric=True)
    finally:
        domain_module.certificate_of = real
    text = []
    for order, count in (("tottime", 45), ("cumulative", 30)):
        out = io.StringIO()
        pstats.Stats(profile, stream=out).sort_stats(order).print_stats(count)
        text.append(out.getvalue())
    Path(out_path).with_suffix(".prof.txt").write_text("\n\n".join(text), encoding="utf-8")
    report["profile_written"] = True


def overhead_of(prepared, alpha_text, kw, real_step, repeats):
    """`(seconds without, seconds with)` of a FAST_HIT step at alpha0, the minimum of `repeats`."""

    best_without = best_with = None
    for _ in range(repeats):
        begun = time.perf_counter()
        real_step(prepared, alpha_text, **kw, parametric=False)
        took = time.perf_counter() - begun
        best_without = took if best_without is None else min(best_without, took)
        begun = time.perf_counter()
        real_step(prepared, alpha_text, **kw, parametric=True)
        took = time.perf_counter() - begun
        best_with = took if best_with is None else min(best_with, took)
    return best_without, best_with


def judge_domain(theorem, prepared, alpha_text, kw, real_step, shares, tally):
    """The theorem on one domain over all shares; returns its failures."""

    base = real_step(prepared, alpha_text, **kw, parametric_exact=True)
    if not theorem.usable(base.result.parametric):
        tally["counts"]["BASE_NOT_USABLE"] += 1
        return []
    alpha0 = Fraction(alpha_text)
    interval = base.result.interval
    failures = []
    for share in shares:
        text = theorem.decimal_text(alpha0 * (1 + Fraction(share)))
        alpha1 = Fraction(text)
        if alpha1 <= 0:
            continue
        other = real_step(prepared, text, **kw, parametric_exact=True)
        if not other.result.is_materialized:
            tally["counts"]["OTHER_REFUSED"] += 1
            continue
        inside = interval is not None and interval.contains_exact(alpha1)
        outcome, found = theorem.theorem_differences(base, other, alpha0, alpha1)
        tally["counts"][outcome + ("_INSIDE" if inside else "_OUTSIDE")] += 1
        if outcome != theorem.JUDGED:
            continue
        if found:
            failures.append({"patch": base.result.batch.patch_domain_id.value, "share": share, "inside": inside, "differences": [str(x) for x in found[:5]], "count": len(found)})
        errors = theorem.float_errors(base, other, alpha0, alpha1)
        if errors is not None and inside:
            tally["position"], tally["uv"] = max(tally["position"], errors[0]), max(tally["uv"], errors[1])
    return failures


def probe(args, tree, report) -> None:
    import cftuv_envelope.materialize.step as step_module
    import parametric_theorem as theorem

    real_step = step_module.step_domain
    captured, run, seconds = capture_press(args, tree, step_module, real_step)
    report.update(press_seconds=seconds, domains=len(run.results), materialized=sum(1 for item in run.results if item.is_materialized), captured=len(captured))
    print(f"FIELD press {args.mesh} domains={report['domains']} captured={len(captured)} seconds={seconds:.2f}", flush=True)
    if args.profile:
        profile_certificates(captured, real_step, report, args.out)
    shares = [float(item) for item in args.shares.split(",") if item]
    markers, tally, failures = Counter(), {"counts": Counter(), "position": 0.0, "uv": 0.0}, []
    spent_without = spent_with = 0.0
    measured = 0
    for prepared, alpha_text, kw, first in (captured if not args.limit else captured[: args.limit]):
        if not first.result.is_materialized:
            continue
        kw = {**kw, "digests": False}
        certificate = first.result.parametric
        for name in ("PARAM_ABSENT",) if certificate is None else (certificate.markers or certificate.approximate or ("AFFINE",)):
            markers[name] += 1
        without, with_certificate = overhead_of(prepared, alpha_text, kw, real_step, args.repeats)
        spent_without, spent_with, measured = spent_without + without, spent_with + with_certificate, measured + 1
        failures += judge_domain(theorem, prepared, alpha_text, kw, real_step, shares, tally)
    report.update(
        markers=dict(markers), counts=dict(tally["counts"]), theorem_failures=failures[:40], theorem_failure_count=len(failures),
        worst_position_error=tally["position"], worst_uv_error=tally["uv"], overhead_seconds_without=spent_without, overhead_seconds_with=spent_with,
        overhead_domains=measured, overhead_ratio=(spent_with / spent_without if spent_without else None),
    )
    print("FIELD", json.dumps({k: report[k] for k in ("markers", "counts", "theorem_failure_count", "worst_position_error", "worst_uv_error", "overhead_ratio")}, default=str), flush=True)


def main() -> int:
    import bpy

    args = _arguments()
    root = Path(args.root).resolve()
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    sys.path.insert(0, str(root / "kernel" / "tests"))
    import blender_tree_ab as tree

    tree.load_tree(root, args.native_path)
    import mpmath  # noqa: F401  (Blender's profile carries them; imported early so a missing one fails here)
    import sympy  # noqa: F401

    report: dict = {"root": str(root), "mesh": args.mesh, "scene_kind": args.scene, "alpha": args.alpha, "blender": bpy.app.version_string, "blend": bpy.data.filepath, "failure": None}
    failure = None
    try:
        report["native"] = tree.verify_native(args.native_path, args.native_prefix)
        probe(args, tree, report)
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
