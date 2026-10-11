"""Field export of the corpus of the native `effective_alpha` port: the real button's cold press on one mesh of a scene, every distinct pair of the resolver in a record.

    blender -b <COPY of the scene>.blend --python-exit-code 1 --python tools/native_effective_alpha_export.py -- \\
        --case building|rounded|two|sagging0987|half_sphere|cover008 [--out <corpus dir>] [--native-path DIR]

The case is one of the built-in recipes of `tools/blender_tree_ab.py` (the mesh, alpha, density, stretch budget and scene of the field A/B). The press is the A/B tool's own cold press (`run_presses`: the controller
of a session, the stored or seam selection, `run_production` in this process: no workers), with the add-on and the extension taken from the tree of the repository and `native_path` ahead of everything. The stage
`effective_alpha` is NOT ordered (the oracle computes every pair), and the recorder wraps `boundary._contact_candidates`, so each pair the resolver asks is recorded once per distinct input with its call count.
The scene is never saved: the SHA-256 of the .blend is taken before and after, and a change fails the export.

Output: `<out>/field/<mesh>.recs.xz` (`native_effective_alpha_corpus` format), `<out>/index.json`. The last line on success: `NATIVE_EFFECTIVE_ALPHA_EXPORT_OK <pairs> <bytes>`.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import sys
import time
import traceback
from pathlib import Path

import bpy

try:
    ROOT = Path(__file__).resolve().parents[1]
except NameError:  # a Blender that does not define __file__ for --python scripts
    ROOT = Path(sys.argv[sys.argv.index("--python") + 1]).resolve().parents[1]
# SNAPSHOT_BYTE_CHECK_SAMPLED_V1: the gates of the tools check every domain whole (the product's button samples).
os.environ["CFTUV_SNAPSHOT_BYTE_CHECK"] = "all"
sys.path.insert(0, str(ROOT / "tools"))


def _arguments():
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", default=str(ROOT))
    parser.add_argument("--case", required=True)
    parser.add_argument("--out", default="")
    parser.add_argument("--native-path", default="")
    parser.add_argument("--presses", default="cold")
    return parser.parse_args(sys.argv[sys.argv.index("--") + 1 :] if "--" in sys.argv else [])


def _sha256(path: str) -> str:
    digest = hashlib.sha256()
    with open(path, "rb") as stream:
        for block in iter(lambda: stream.read(1 << 20), b""):
            digest.update(block)
    return digest.hexdigest()


def main() -> int:
    options = _arguments()
    import blender_tree_ab as tree_ab

    case = tree_ab.BUILT_IN_CASES[options.case]
    scene_path = bpy.data.filepath
    scene_sha = _sha256(scene_path)
    root = Path(options.root).resolve()
    tree_ab.load_tree(root, options.native_path)
    import cftuv_envelope.materialize.step  # noqa: F401
    import cftuv_envelope.wavefront  # noqa: F401
    import mpmath  # noqa: F401
    import sympy  # noqa: F401
    from cftuv.envelope_debug_session import WINDOW_MANAGER_SESSION_ATTRIBUTE, EnvelopeDebugSessionController

    import native_effective_alpha_corpus as nec  # after the tree is loaded: the recorder patches the module the add-on uses

    args = argparse.Namespace(
        scene_kind=case.scene, mesh=case.mesh, alpha=case.alpha, density=case.density, stretch=case.stretch, offset=case.offset, workers=0, gc_pause="auto",
        step_width=tree_ab.STEP_WIDTH, presses=options.presses,
    )
    presses = tree_ab.parse_presses(args.presses, args.alpha, args.step_width)
    scene, obj, settings, mesh_settings = tree_ab._open_case(args)
    mesh_settings.kernel_backend = "NATIVE"
    if args.offset >= 0:
        mesh_settings.offset = args.offset
    controller = getattr(bpy.context.window_manager, WINDOW_MANAGER_SESSION_ATTRIBUTE, None)
    if not isinstance(controller, EnvelopeDebugSessionController):
        controller = EnvelopeDebugSessionController()
        setattr(bpy.context.window_manager, WINDOW_MANAGER_SESSION_ATTRIBUTE, controller)
    state = {
        "keys": [], "labels": [p[0] for p in presses], "kinds": [p[1] for p in presses], "alphas": {p[0]: p[2] for p in presses},
        "obj": obj, "settings": settings, "mesh_settings": mesh_settings, "controller": controller,
    }
    report: dict = {}
    out = (Path(options.out) if options.out else nec.corpus_directory()) / "field"
    recorder = nec.Recorder(f"field:{case.mesh}")
    failure = None
    started = time.perf_counter()
    try:
        with recorder.installed():
            tree_ab.run_presses(args, report, state)
    except Exception:  # noqa: BLE001 - the reason goes to the report and the exit code
        failure = traceback.format_exc()
        print(failure, flush=True)
    seconds = time.perf_counter() - started
    document = recorder.document()
    target = out / f"{case.mesh}.recs.xz"
    if target.exists() and failure is None:  # a second press of the same mesh (another case) adds its pairs
        document = nec.merge_documents([nec.read_document(target), document])
    size = nec.write_document(target, document)
    shape = nec.shape_of(document)
    from cftuv.envelope_domain_pool import shutdown_domain_pool

    shutdown_domain_pool()
    after = _sha256(scene_path)
    if after != scene_sha:
        failure = (failure or "") + f"\nSCENE_CHANGED {scene_sha} -> {after}"
    nec.write_index(out.parent, {"blender": bpy.app.version_string, "scene": scene_path, "scene_sha256": scene_sha, "field": {case.mesh: {**shape, "bytes": size, "seconds": round(seconds, 1), "case": options.case, "press": report.get("presses", {}).get("cold", {}).get("outcomes")}}})
    print(json.dumps({"case": options.case, "mesh": case.mesh, **shape, "seconds": round(seconds, 1)}), flush=True)
    if failure is not None or not document["pairs"]:
        print(f"NATIVE_EFFECTIVE_ALPHA_EXPORT_FAILED failure={failure is not None} pairs={len(document['pairs'])}")
        return 1
    print(f"NATIVE_EFFECTIVE_ALPHA_EXPORT_OK {len(document['pairs'])} {size}")
    return 0


if __name__ == "__main__":
    code = main()
    if code:
        raise SystemExit(code)
