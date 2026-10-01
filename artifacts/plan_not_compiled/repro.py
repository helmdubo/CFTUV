"""Воспроизведение PLAN_IS_NOT_COMPILED вне Blender по выгруженным фикстурам.

  PYTHONSAFEPATH=1 PYTHONPATH=kernel/src python artifacts/plan_not_compiled/repro.py [mesh_2|building ...] [--density 4]

На каждый (патч, плотность) зовёт `compile_reference_envelopes` и печатает исход
и ПЕРВЫЙ ввод в игру неразрешимости: исход `ReferenceGeometryError` либо
`CertifiedPredicateUndecidable` (стек создания, файл:строка).
"""
from __future__ import annotations

import json
import sys
import time
import traceback
from pathlib import Path

HERE = Path(__file__).resolve().parent

from cftuv_envelope import AnalysisSnapshotCodecV1, DecalRequestCodecV1  # noqa: E402
from cftuv_envelope.reference.compile import compile_reference_envelopes  # noqa: E402
from cftuv_envelope.reference import planar_types, direction_binding, adaptive_density_fan  # noqa: E402

CREATED = []


def _hook():
    for cls in (adaptive_density_fan.AdaptiveDensityFanInvalid, planar_types.CertifiedPredicateUndecidable, direction_binding.DirectionBindingCertificateUnproven):
        original = cls.__init__

        def init(self, *args, __original=original, **kwargs):
            CREATED.append(
                (
                    type(self).__name__,
                    str(args[0])[:600] if args else "",
                    [f"{Path(f.filename).name}:{f.lineno}:{f.name}" for f in traceback.extract_stack()[-9:-1]],
                )
            )
            __original(self, *args, **kwargs)

        cls.__init__ = init


def run(mesh: str, patch: int, density: int):
    base = HERE / mesh / f"patch_{patch}"
    snapshot = AnalysisSnapshotCodecV1.loads((base / "analysis_snapshot.json").read_bytes())
    request = DecalRequestCodecV1.loads((base / f"decal_request_d{density}.json").read_bytes())
    domain_id = json.loads((HERE / mesh / "manifest.json").read_text(encoding="utf-8"))["domains"][str(patch)]
    CREATED.clear()
    started = time.perf_counter()
    result = compile_reference_envelopes(snapshot, request)
    seconds = time.perf_counter() - started
    message = result.diagnostics[0].message if result.diagnostics else ""
    return result.outcome.value, message, seconds, list(CREATED)


def main():
    args = sys.argv[1:]
    density = 4
    if "--density" in args:
        i = args.index("--density")
        density = int(args[i + 1])
        del args[i:i + 2]
    meshes = args or ["mesh_2", "building"]
    _hook()
    for mesh in meshes:
        manifest = json.loads((HERE / mesh / "manifest.json").read_text(encoding="utf-8"))
        for patch in manifest["domains"]:
            outcome, message, seconds, created = run(mesh, int(patch), density)
            print(f"{mesh} p{patch} d{density} {outcome} {seconds:.2f}s :: {message[:300]}")
            for item in created[:3]:
                print("    ", item)


if __name__ == "__main__":
    main()
