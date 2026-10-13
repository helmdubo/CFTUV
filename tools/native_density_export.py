"""Реальная headless-кнопка поля под рекордером Density; исходная сцена не сохраняется.

blender -b COPY.blend --python-exit-code 1 --python tools/native_density_export.py -- \
    --case building --out E:/cftuv_native_corpus/density --native-path DIR

По native_effective_alpha_export: run_presses, workers=0, cold press, hash сцены
до/после. Любой отказ/исключение сохраняется; исключение пресса завершает процесс
ненулевым кодом. --density позволяет повторить case на другой плотности.
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

ROOT = Path(__file__).resolve().parents[1]
os.environ["CFTUV_SNAPSHOT_BYTE_CHECK"] = "all"
sys.path.insert(0, str(ROOT / "tools"))


def _arguments():
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", default=str(ROOT))
    parser.add_argument("--case", required=True)
    parser.add_argument("--out", default="")
    parser.add_argument("--native-path", default="")
    parser.add_argument("--density", type=int)
    return parser.parse_args(sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else [])


def _sha256(path):
    digest = hashlib.sha256()
    with open(path, "rb") as stream:
        for block in iter(lambda: stream.read(1 << 20), b""):
            digest.update(block)
    return digest.hexdigest()


def _press_state(tree_ab, case, options):
    from cftuv.envelope_debug_session import WINDOW_MANAGER_SESSION_ATTRIBUTE, EnvelopeDebugSessionController

    args = argparse.Namespace(
        scene_kind=case.scene, mesh=case.mesh, alpha=case.alpha,
        density=case.density if options.density is None else options.density,
        stretch=case.stretch, offset=case.offset, workers=0, gc_pause="auto",
        step_width=tree_ab.STEP_WIDTH, presses="cold",
    )
    presses = tree_ab.parse_presses(args.presses, args.alpha, args.step_width)
    _, obj, settings, mesh_settings = tree_ab._open_case(args)
    mesh_settings.kernel_backend = "NATIVE"
    if args.offset >= 0:
        mesh_settings.offset = args.offset
    controller = getattr(bpy.context.window_manager, WINDOW_MANAGER_SESSION_ATTRIBUTE, None)
    if not isinstance(controller, EnvelopeDebugSessionController):
        controller = EnvelopeDebugSessionController()
        setattr(bpy.context.window_manager, WINDOW_MANAGER_SESSION_ATTRIBUTE, controller)
    return args, {
        "keys": [], "labels": [p[0] for p in presses], "kinds": [p[1] for p in presses],
        "alphas": {p[0]: p[2] for p in presses}, "obj": obj, "settings": settings,
        "mesh_settings": mesh_settings, "controller": controller,
    }


def main() -> int:
    options = _arguments()
    import blender_tree_ab as tree_ab

    case = tree_ab.BUILT_IN_CASES[options.case]
    scene_path = bpy.data.filepath
    scene_sha = _sha256(scene_path)
    tree_ab.load_tree(Path(options.root).resolve(), options.native_path)
    import native_density_corpus as corpus

    args, state = _press_state(tree_ab, case, options)
    recorder = corpus.Recorder(f"field:{case.mesh}:d{args.density}")
    report, failure = {}, None
    started = time.perf_counter()
    try:
        with recorder.installed():
            tree_ab.run_presses(args, report, state)
    except Exception:  # noqa: BLE001 - traceback и ненулевой exit, а не успешно пустой корпус
        failure = traceback.format_exc()
        print(failure, flush=True)
    seconds = time.perf_counter() - started
    from cftuv.envelope_domain_pool import shutdown_domain_pool

    shutdown_domain_pool()
    after = _sha256(scene_path)
    if after != scene_sha:
        failure = (failure or "") + f"\nSCENE_CHANGED {scene_sha} -> {after}"
    document = recorder.document()
    out = Path(options.out) if options.out else corpus.corpus_directory()
    target = out / "field" / f"{case.mesh}_d{args.density}.recs.xz"
    size = corpus.write_document(target, document)
    receipt = {"case": options.case, "mesh": case.mesh, "density": args.density,
               "scene": scene_path, "scene_sha256": scene_sha, "scene_sha256_after": after,
               "blender": bpy.app.version_string, "bytes": size, "seconds": seconds,
               "failure": failure, "press": report, **corpus.shape_of(document)}
    target.with_suffix(".receipt.json").write_text(json.dumps(receipt, indent=2, default=str), encoding="utf-8")
    print("NATIVE_DENSITY_EXPORT " + corpus.json_key({key: value for key, value in receipt.items() if key != "press"}), flush=True)
    return 1 if failure is not None or not document["records"] else 0


if __name__ == "__main__":
    code = main()
    if code:
        raise SystemExit(code)
