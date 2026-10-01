"""Перепись углов полигона очереди: у скольких ВЫПУКЛЫХ / ВОГНУТЫХ углов есть веер.

  PYTHONSAFEPATH=1 python artifacts/fan_consistency/reflex_convex_census.py <mesh> <density>

Полигон — решётка домена (PolygonV1 после привязки). Внутренность слева от рёбер (внешняя петля против
часовой, дыры по часовой): вершина вогнута, если cross(вход, выход) < 0 в метрике карты. Вееры — узлы из
ключей владения скрытых опор. Вывод: счётчики по классам угла.
"""
from __future__ import annotations

import json
import sys
from collections import Counter
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import _paths  # noqa: E402

_paths.add_kernel_paths()

import fanlib  # noqa: E402
from cftuv_envelope import AnalysisSnapshotCodecV1, DecalRequestCodecV1  # noqa: E402
from cftuv_envelope.wavefront import prepare_conveyor  # noqa: E402


def classify(view, p0, p1, p2):
    a = (p1[0] - p0[0], p1[1] - p0[1])
    b = (p2[0] - p1[0], p2[1] - p1[1])
    cross = a[0] * b[1] - a[1] * b[0]
    if cross == 0:
        return "COLLINEAR"
    dot = view.gdot(a, b)
    # знак cross в метрике карты равен знаку orientation-детерминанта базиса (положителен для CCW-карты)
    kind = "REFLEX" if cross < 0 else "CONVEX"
    right = "RIGHT" if dot == 0 else "OTHER"
    return f"{kind}_{right}"


def main() -> None:
    mesh, density = sys.argv[1], int(sys.argv[2])
    manifest = json.loads((_paths.mesh_dir(mesh) / "manifest.json").read_text(encoding="utf-8"))
    total = Counter()
    for patch in sorted(manifest["domains"], key=int):
        base = _paths.mesh_dir(mesh) / f"patch_{patch}"
        snap = AnalysisSnapshotCodecV1.loads((base / "analysis_snapshot.json").read_bytes())
        req = DecalRequestCodecV1.loads((base / f"decal_request_d{density}.json").read_bytes())
        prep = prepare_conveyor(snap, req)
        if prep.outcome.value != "EXACT":
            continue
        view = fanlib.DomainView(snap, prep)
        fan_nodes = {(key[0], key[1]) for r in prep.regions for key, _ in r.owner_by_edge if len(key) == 5}
        for region in prep.regions:
            polygon = region.bridge.polygon
            for loop in polygon.loops:
                pts = loop.points
                n = len(pts)
                for i in range(n):
                    kind = classify(view, pts[i - 1], pts[i], pts[(i + 1) % n])
                    has_fan = pts[i] in fan_nodes
                    total[(kind, "FAN" if has_fan else "NO_FAN")] += 1
    print(mesh, f"d{density}")
    for key, value in sorted(total.items()):
        print("  ", key, value)


main()
