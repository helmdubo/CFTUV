"""Сырые углы меша в Blender (float32 как есть) против классов решётки: честная ли это геометрия?

  blender.exe -b E:\\testscene.blend --python artifacts/fan_consistency/blender_raw_angles.py -- <results_dir> <mesh> <density>

Читает перечень `fans_<mesh>_d<density>.json` (enumerate_fans.py), для каждого веерного угла берёт у ЖИВОГО меша
вершину и её соседей по петле (`prev`/`next` из перечня), считает угол между рёбрами в object-space в float64 и
печатает отклонение от 90 градусов. Сцену не сохраняет и не меняет.
"""
from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import bmesh  # noqa: E402
import bpy  # noqa: E402


def main() -> None:
    argv = sys.argv[sys.argv.index("--") + 1:]
    results_dir, mesh, density = Path(argv[0]), argv[1], int(argv[2])
    rows = json.loads((results_dir / f"fans_{mesh.replace('.', '_')}_d{density}.json").read_text(encoding="utf-8"))
    obj = bpy.data.objects[mesh]
    bm = bmesh.new()
    bm.from_mesh(obj.data)
    bm.verts.ensure_lookup_table()
    groups: dict = {}
    signs: dict = {}
    pairs: list = []
    for row in rows:
        for c in row["corners"]:
            v = bm.verts[c["vertex"]].co
            p = bm.verts[c["prev"]].co
            n = bm.verts[c["next"]].co
            a = (v - p).normalized()
            b = (n - v).normalized()
            turn = math.degrees(math.acos(max(-1.0, min(1.0, a.dot(b)))))
            dev = turn - 90.0
            if abs(dev) > 0.5:
                continue
            eval_class = ("RAWK_EXACT/" if c["raw_exact_half"] else "RAWK_OFF/") + ("EXACT" if c["eval_dot0"] else ("ABOVE" if abs(c["eval_turn_deg"]) > 90.0 else "BELOW"))
            groups.setdefault(eval_class, []).append(dev)
            if c["raw_exact_half"] and not c["eval_dot0"]:
                eval_dev = abs(c["eval_turn_deg"]) - 90.0
                key = "agree" if (dev == 0.0 and False) else (
                    "raw_zero" if dev == 0.0 else ("agree" if (dev > 0) == (eval_dev > 0) else "disagree")
                )
                signs[key] = signs.get(key, 0) + 1
                pairs.append((dev, eval_dev))
    print("RAWFLOAT sign agreement raw-float vs evaluation-lattice (kernel-raw-exact, eval-noisy):", signs)
    if pairs:
        n = len(pairs)
        mx = sum(p[0] for p in pairs) / n
        my = sum(p[1] for p in pairs) / n
        sxx = sum((p[0] - mx) ** 2 for p in pairs)
        syy = sum((p[1] - my) ** 2 for p in pairs)
        sxy = sum((p[0] - mx) * (p[1] - my) for p in pairs)
        print("RAWFLOAT pearson(raw_float_dev, eval_dev) =", (sxy / math.sqrt(sxx * syy)) if sxx and syy else None, "n =", n)
    for name, devs in sorted(groups.items()):
        print(
            f"RAWFLOAT {mesh} class={name} n={len(devs)} raw_dev_deg min={min(devs):+.2e} max={max(devs):+.2e} "
            f"mean_abs={sum(abs(d) for d in devs) / len(devs):.2e}"
        )
    bm.free()


main()
