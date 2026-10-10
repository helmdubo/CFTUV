"""Field export of the corpus of the native `snap_embedding` port: the real button's export phase on the scene meshes, every distinct call of the leaf in a record.

    blender -b E:/testScene.blend --python-exit-code 1 --python tools/native_embedding_export.py -- \\
        [--meshes building,rounded_wall_noise_top,sagging_wall,2,half_sphere] [--alpha 0.2239] [--stretches 42,12] [--density 2] [--out <corpus dir>] [--full]

The leaf `_embedding._compute_source_snap_embedding_certificate` runs while the host exports a patch metric (`get_patch_metric`, inside `_scan` of `run_production`, sequential:
no workers), so the press stops there: the cold preparation of a domain (the skeleton, the coverage, the materialization) is replaced by a named refusal unless `--full` asks for the whole press.
The memo of the certificate is off, so every call is computed and recorded (once per distinct input, with a call count). One cold session per (mesh, stretch budget): the budget is part of the
cache key of the metric and changes the rung of the ladder that is tried. The scene is never saved; the add-on comes from the tree of the repository.

Output: `<out>/field/<mesh>.recs.xz` (`native_embedding_corpus` format), `<out>/index.json`. The last line on success: `NATIVE_EMBEDDING_EXPORT_OK <records> <bytes>`.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
import traceback
from pathlib import Path

import bpy

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))
DEFAULT_MESHES = "building,rounded_wall_noise_top,sagging_wall,2,half_sphere"


def _arguments():
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", default=str(ROOT))
    parser.add_argument("--meshes", default=DEFAULT_MESHES)
    parser.add_argument("--alpha", type=float, default=0.2239)
    parser.add_argument("--stretches", default="42,12")
    parser.add_argument("--density", type=int, default=2)
    parser.add_argument("--stretch", type=int, default=42)
    parser.add_argument("--out", default="")
    parser.add_argument("--full", action="store_true")
    return parser.parse_args(sys.argv[sys.argv.index("--") + 1 :] if "--" in sys.argv else [])


class _Context:
    """The only thing `native_corpus_export._open_mesh` asks of a recorder: a dict to note the mesh in."""

    context: dict = {}


def _stop_after_export(export_module) -> None:
    """The cold preparation is where a domain would start computing: it raises, the host names the domain refused, and the metric (already built by `_scan`) stays in the session."""

    def refuse(snapshot, request):
        raise RuntimeError("native_embedding_export: the press stops after the export phase")

    export_module.prepare_for_production = refuse


def _press(ctx: dict, name: str, alpha: float, stretch: int) -> None:
    from cftuv.envelope_request_policy import envelope_stretch_budget

    base = ctx["base"]
    opened = base._open_mesh(ctx, name, fresh=True)
    ctx["run_production"](
        ctx["controller"], opened["bundle"], frozenset(opened["selected"]), alpha,
        source_object_key=opened["key"], source_data_key=opened["data_key"], density=ctx["args"].density,
        developable_stretch_budget=envelope_stretch_budget(stretch), workers=0, kernel_backend="PYTHON",
        embedding_backend="PYTHON",  # the corpus records the ORACLE's calls: the default of the stage (Native) would bypass the recorder
    )


def main() -> int:
    args = _arguments()
    import native_corpus_export as base

    ctx = base._context(args)
    ctx.update(base=base, recorder=_Context())
    import native_embedding_corpus as nec  # after the tree is loaded: the recorder patches the module the add-on uses

    out = (Path(args.out) if args.out else nec.corpus_directory()) / "field"
    out.mkdir(parents=True, exist_ok=True)
    if not args.full:
        _stop_after_export(ctx["export_module"])
    summary: dict = {}
    failure = None
    total_bytes = 0
    for name in [item.strip() for item in args.meshes.split(",") if item.strip()]:
        if name not in bpy.data.objects:
            print(f"MESH_ABSENT {name}", flush=True)
            summary[name] = {"absent": True}
            continue
        recorder = nec.Recorder(f"field:{name}")
        started = time.perf_counter()
        try:
            with recorder.installed():
                for stretch in [int(item) for item in args.stretches.split(",") if item.strip()]:
                    before = len(recorder.records)
                    _press(ctx, name, args.alpha, stretch)
                    print(f"  {name} stretch={stretch}: +{len(recorder.records) - before} records, {recorder.calls} calls, {time.perf_counter() - started:.1f}s", flush=True)
        except Exception:  # noqa: BLE001 - the reason goes to the report and the exit code
            failure = traceback.format_exc()
            print(failure, flush=True)
        records = list(recorder.records.values())
        total_bytes += nec.write_records(out / f"{name}.recs.xz", records)
        summary[name] = {**nec.shape_of(records), "seconds": round(time.perf_counter() - started, 1)}
        print(f"{name}: {json.dumps(summary[name])}", flush=True)
    from cftuv.envelope_domain_pool import shutdown_domain_pool

    shutdown_domain_pool()
    nec.write_index(out.parent, {"blender": bpy.app.version_string, "scene": bpy.data.filepath, "field": summary, "full_press": args.full})
    records = sum(row.get("records", 0) for row in summary.values())
    if failure is not None or not records:
        print(f"NATIVE_EMBEDDING_EXPORT_FAILED failure={failure is not None} records={records}")
        return 1
    print(f"NATIVE_EMBEDDING_EXPORT_OK {records} {total_bytes}")
    return 0


if __name__ == "__main__":
    code = main()
    if code:
        raise SystemExit(code)
