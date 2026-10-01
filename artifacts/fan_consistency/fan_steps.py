"""Фактические углы шагов веера на полигоне очереди (в метрике карты): равномерны ли шаги по классам.

  PYTHONSAFEPATH=1 python artifacts/fan_consistency/fan_steps.py <mesh> <density> [patch,patch...]

Для каждого веера берутся направления: входящее ребро, лучи скрытых опор (несущая a*x+b*y=c даёт направление
(b, -a)), исходящее ребро — все по узлам решётки полигона. Шаг — угол между соседними направлениями в метрике
карты (acos, градусы, float: это ОТЧЁТ, не решение). Печатает по классам оценку: сколько шагов, min/max, и
наибольшее отклонение шага от среднего.
"""
from __future__ import annotations

import json
import math
import sys
from collections import defaultdict
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import _paths  # noqa: E402

_paths.add_kernel_paths()

import fanlib  # noqa: E402
from cftuv_envelope import AnalysisSnapshotCodecV1, DecalRequestCodecV1  # noqa: E402
from cftuv_envelope.wavefront import prepare_conveyor  # noqa: E402


def angle(view, u, v) -> float:
    c = float(view.gdot(u, v)) / math.sqrt(float(view.gdot(u, u)) * float(view.gdot(v, v)))
    return math.degrees(math.acos(max(-1.0, min(1.0, c))))


def main() -> None:
    mesh, density = sys.argv[1], int(sys.argv[2])
    manifest = json.loads((_paths.mesh_dir(mesh) / "manifest.json").read_text(encoding="utf-8"))
    patches = sorted(manifest["domains"], key=int)
    if len(sys.argv) > 3 and not sys.argv[3].startswith("--"):
        patches = [p for p in patches if p in sys.argv[3].split(",")]
    classes = defaultdict(list)
    for patch in patches:
        base = _paths.mesh_dir(mesh) / f"patch_{patch}"
        snap = AnalysisSnapshotCodecV1.loads((base / "analysis_snapshot.json").read_bytes())
        req = DecalRequestCodecV1.loads((base / f"decal_request_d{density}.json").read_bytes())
        prep = prepare_conveyor(snap, req)
        if prep.outcome.value != "EXACT":
            continue
        view = fanlib.DomainView(snap, prep)
        for region in prep.regions:
            polygon = region.bridge.polygon
            fans = {fan.point: fan for fan in polygon.vertex_fans}
            for loop in polygon.loops:
                pts = loop.points
                n = len(pts)
                for i, node in enumerate(pts):
                    fan = fans.get(node)
                    if fan is None:
                        continue
                    prev, nxt = pts[i - 1], pts[(i + 1) % n]
                    d_in = (node[0] - prev[0], node[1] - prev[1])
                    d_out = (nxt[0] - node[0], nxt[1] - node[1])
                    dirs = [d_in]
                    for s in fan.supports:
                        ray = (s.normal_y, -s.normal_x)
                        if view.gdot(ray, dirs[-1]) < 0:
                            ray = (-ray[0], -ray[1])
                        dirs.append(ray)
                    dirs.append(d_out)
                    steps = [angle(view, dirs[k], dirs[k + 1]) for k in range(len(dirs) - 1)]
                    dot_exact = view.gdot(d_in, d_out) == 0
                    turn = angle(view, d_in, d_out)
                    sign = "EXACT" if dot_exact else ("ABOVE" if turn > 90.0 else "BELOW")
                    classes[(f"H={len(fan.supports)}", sign)].append((int(patch), node, steps, turn))
    if "--all" in sys.argv:
        for key, items in sorted(classes.items()):
            for patch, node, steps, turn in items:
                print(f"  {key} patch {patch} node {node} steps={[round(x, 3) for x in steps]} turn={turn:.5f}")
    for key, items in sorted(classes.items()):
        all_steps = [s for _p, _n, st, _t in items for s in st]
        spread = max(max(st) - min(st) for _p, _n, st, _t in items)
        print(
            f"{mesh} d{density} {key[0]} eval={key[1]:5s} fans={len(items):3d} steps/fan={len(items[0][2])} "
            f"step min={min(all_steps):.5f} max={max(all_steps):.5f} max_intra_fan_spread={spread:.5f} deg  "
            f"e.g. patch {items[0][0]} steps={[round(x, 5) for x in items[0][2]]} turn={items[0][3]:.5f}"
        )


main()
