"""Cold field press recording raw-turn calls; Blender запускает владелец, сцена не сохраняется.

blender -b FROZEN.blend --python-exit-code 1 --python tools/native_density_turn_export.py -- \
    --case building --density 2 --out D:/CFTUV-r3-density-corpus/turn-field --native-path DIR

Подготовка пресса переиспользует frozen native_density_export._press_state.
Нулевая граница даёт namedcoverage receipt и exit1, не успешный пустой корпус.
"""

from __future__ import annotations

import argparse
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
    parser.add_argument("--out", default="D:/CFTUV-r3-density-corpus/turn-field")
    parser.add_argument("--native-path", default="")
    parser.add_argument("--density", type=int)
    return parser.parse_args(sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else [])


def main():
    options = _arguments()
    import blender_tree_ab as tree_ab
    from native_density_export import _press_state, _sha256

    scene = bpy.data.filepath
    before = _sha256(scene)
    tree_ab.load_tree(Path(options.root).resolve(), options.native_path)
    import native_density_turn_corpus as corpus

    case = tree_ab.BUILT_IN_CASES[options.case]
    args, state = _press_state(tree_ab, case, options)
    recorder = corpus.Recorder(f"field_turn:{case.mesh}:d{args.density}")
    report, failure = {}, None
    started = time.perf_counter()
    try:
        with recorder.installed():
            tree_ab.run_presses(args, report, state)
    except Exception:  # noqa: BLE001 - traceback и exit1, без маскировки исключения пресса
        failure = traceback.format_exc()
        print(failure, flush=True)
    from cftuv.envelope_domain_pool import shutdown_domain_pool

    shutdown_domain_pool()
    after = _sha256(scene)
    if after != before:
        failure = (failure or "") + f"\nSCENE_CHANGED {before} -> {after}"
    document = recorder.document()
    path = Path(options.out) / "field" / f"{case.mesh}_d{args.density}.turns.xz"
    size = corpus.write_document(path, document)
    coverage = "TURN_BOUNDARY_RECORDED_V1" if recorder.rows else "NO_DENSITY_TURN_CALLS_V1"
    receipt = {"case": options.case, "mesh": case.mesh, "density": args.density,
               "scene": scene, "scene_sha256": before, "scene_sha256_after": after,
               "seconds": time.perf_counter() - started, "failure": failure,
               "coverage": coverage, "bytes": size, "press": report, **corpus.shape_of(document)}
    path.with_suffix(".receipt.json").write_text(json.dumps(receipt, indent=2, default=str), encoding="utf-8")
    print("NATIVE_DENSITY_TURN_EXPORT " + corpus.json_key({key: value for key, value in receipt.items() if key != "press"}), flush=True)
    return 1 if failure is not None or not recorder.rows else 0


if __name__ == "__main__":
    code = main()
    if code:
        raise SystemExit(code)
