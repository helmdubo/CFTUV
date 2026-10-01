"""Что именно упирается в `_termination_boxes`: идеальный веер каждого отказника.

  PYTHONSAFEPATH=1 PYTHONPATH=kernel/src python artifacts/plan_not_compiled/diag_fan.py [--density 4] [mesh ...]

На каждый домен перехватывает вход `_termination_boxes` (метрика, идеальные
ковекторы, ориентация, q, окна) и печатает по соседним парам идеального веера:
точный cos^2 угла (sympy, `nsimplify`), угол в градусах и ТОЧНЫЙ ответ
`_subturn_boundary` (== 0 на границе допуска): стоит ли пара РОВНО на пределе
Delta_max. Если стоит, у веера нет ни одного рационального допустимого
возмущения, и положительная termination-коробка не существует по построению.
"""
from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import sympy as sp

from cftuv_envelope import AnalysisSnapshotCodecV1, DecalRequestCodecV1
from cftuv_envelope.reference import adaptive_density_fan as fan
from cftuv_envelope.reference.compile import compile_reference_envelopes

HERE = Path(__file__).resolve().parent
CAPTURED = []


def _wrap():
    original = fan._termination_boxes

    def wrapper(metric, ideal, orientation, q, records):
        CAPTURED.append((metric, ideal, orientation, q, records))
        return original(metric, ideal, orientation, q, records)

    fan._termination_boxes = wrapper


def analyse(metric, ideal, q):
    out = []
    for index in range(len(ideal) - 1):
        left, right = ideal[index], ideal[index + 1]
        dot = fan._dual_dot(metric, left, right)
        norm = fan._dual_dot(metric, left, left) * fan._dual_dot(metric, right, right)
        cos2 = sp.nsimplify(sp.simplify(dot * dot / norm))
        cross = fan._oriented_cross(metric, left, right)
        angle = math.degrees(math.atan2(float(sp.N(sp.sqrt(sp.simplify(norm - dot * dot)), 30)), float(sp.N(dot, 30))))
        residual = sp.simplify(4 * dot * dot - 3 * norm) if q == 6 else None
        zero_symbolic = residual == 0 if residual is not None else None
        zero_numeric_80 = abs(sp.N(4 * dot * dot - 3 * norm, 80)) < sp.Float("1e-70", 80) if q == 6 else None
        out.append(
            (
                index,
                f"symbolic_zero={zero_symbolic} numeric80_zero={zero_numeric_80}",
                f"{angle:.9f}deg",
                str(cos2),
                "BOUNDARY" if fan._subturn_boundary(metric, left, right, q) else ("ok" if fan._subturn(metric, left, right, q) else "OVER"),
            )
        )
    return out


def main():
    args = sys.argv[1:]
    density = 4
    if "--density" in args:
        i = args.index("--density")
        density = int(args[i + 1])
        del args[i:i + 2]
    only = None
    if "--only" in args:
        i = args.index("--only")
        only = {x for x in args[i + 1].split(",")}
        del args[i:i + 2]
    meshes = args or ["mesh_2", "building"]
    _wrap()
    for mesh in meshes:
        manifest = json.loads((HERE / mesh / "manifest.json").read_text(encoding="utf-8"))
        for patch in manifest["domains"]:
            if only is not None and patch not in only:
                continue
            base = HERE / mesh / f"patch_{patch}"
            snapshot = AnalysisSnapshotCodecV1.loads((base / "analysis_snapshot.json").read_bytes())
            request = DecalRequestCodecV1.loads((base / f"decal_request_d{density}.json").read_bytes())
            CAPTURED.clear()
            result = compile_reference_envelopes(snapshot, request)
            print(f"== {mesh} p{patch} d{density} {result.outcome.value} calls={len(CAPTURED)}")
            for call in CAPTURED:
                metric, ideal, orientation, q, records = call
                rows = analyse(metric, ideal, q)
                print(f"   q={q} orientation={orientation.name} ideal_len={len(ideal)}")
                for row in rows:
                    print("     ", row)


if __name__ == "__main__":
    main()
