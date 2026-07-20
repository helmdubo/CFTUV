"""F0-lite: интегральный замер кадра GPU preview (headless-часть).

Меряет СНАРУЖИ evaluator'а split кадра display-adapter пути:
`evaluate_patch_voronoi_plan` (полный evaluator) против batch-build
(`build_preview_buffers`). Depsgraph/redraw-остаток измерим только в
живом Blender (ручная MCP-сессия) — колонка оставлена в отчёте пустой.

Запуск: python artifacts/verify_decal_gpu_preview.py
Выход: JSON в stdout (median/p95 по кадрам на фикстуру и ширину).
"""

from __future__ import annotations

import json
import statistics
from math import cos, pi, sin
from pathlib import Path
from time import perf_counter
import sys

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

try:
    from mathutils import Vector  # noqa: E402
except ModuleNotFoundError:  # headless вне Blender: стабы тестов
    import tests.conftest  # noqa: E402,F401

    from mathutils import Vector  # noqa: E402

from cftuv.decal_voronoi import (  # noqa: E402
    compile_patch_voronoi_plan,
    evaluate_patch_voronoi_plan,
)
from cftuv.decal_gpu_preview import build_preview_buffers  # noqa: E402
from cftuv.model import (  # noqa: E402
    BoundaryChain,
    BoundaryLoop,
    PatchGraph,
    PatchNode,
)

_WIDTHS = (1.0, 2.0, 3.0, 4.0)
_REPEATS = 7


def _silhouette_graph():
    """Планарный силуэт (углы MITER/FAN/KITE) — фикстура S1-тестов."""

    pts2d = [
        (-12.0, 0.0),
        (12.0, 0.0),
        (12.0, 4.0),
        (4.0, 4.0),
        (2.5, 8.0),
        (-1.0, 4.0),
        (-12.0, 4.0),
    ]
    points = [Vector((x, y, 0.0)) for x, y in pts2d]
    node = PatchNode(
        patch_id=0,
        face_indices=[0],
        centroid=sum(points, Vector((0.0, 0.0, 0.0))) / len(points),
        normal=Vector((0.0, 0.0, 1.0)),
        basis_u=Vector((1.0, 0.0, 0.0)),
        basis_v=Vector((0.0, 1.0, 0.0)),
    )
    node.mesh_verts = points
    node.mesh_tris = [
        (0, 1, 2), (0, 2, 3), (0, 3, 5), (3, 4, 5), (0, 5, 6),
    ]
    chains = [([2, 3], 60), ([3, 4], 61), ([4, 5], 62), ([5, 6], 63)]
    node.boundary_loops = [
        BoundaryLoop(
            chains=[
                BoundaryChain(
                    vert_indices=pair,
                    vert_cos=[points[pair[0]], points[pair[1]]],
                    edge_indices=[edge],
                )
                for pair, edge in chains
            ]
        )
    ]
    graph = PatchGraph()
    graph.add_node(node)
    return graph, [60, 61, 62, 63]


def _fan_wall_graph(segments=24):
    """Плотная зигзаг-кромка: нагрузочная фикстура (много углов)."""

    points = []
    chains = []
    edge_indices = []
    for index in range(segments + 1):
        angle = pi * index / segments
        points.append(
            Vector(
                (
                    10.0 * index / segments,
                    4.0 + (0.8 if index % 2 else 0.0) * sin(angle + 0.3),
                    0.0,
                )
            )
        )
    base = len(points)
    points.append(Vector((10.0, 0.0, 0.0)))
    points.append(Vector((0.0, 0.0, 0.0)))
    node = PatchNode(
        patch_id=0,
        face_indices=[0],
        centroid=sum(points, Vector((0.0, 0.0, 0.0))) / len(points),
        normal=Vector((0.0, 0.0, 1.0)),
        basis_u=Vector((1.0, 0.0, 0.0)),
        basis_v=Vector((0.0, 1.0, 0.0)),
    )
    node.mesh_verts = points
    node.mesh_tris = [
        (index, index + 1, base) for index in range(segments)
    ] + [(0, base, base + 1)]
    for index in range(segments):
        edge = 100 + index
        chains.append(
            BoundaryChain(
                vert_indices=[index, index + 1],
                vert_cos=[points[index], points[index + 1]],
                edge_indices=[edge],
            )
        )
        edge_indices.append(edge)
    node.boundary_loops = [BoundaryLoop(chains=chains)]
    graph = PatchGraph()
    graph.add_node(node)
    return graph, edge_indices


def _measure(graph, edges):
    plan = compile_patch_voronoi_plan(graph, edges, offset=0.01)
    rows = []
    for width in _WIDTHS:
        eval_ms = []
        batch_ms = []
        face_counts = []
        tri_counts = []
        for _repeat in range(_REPEATS):
            started = perf_counter()
            faces = evaluate_patch_voronoi_plan(
                plan, width=width, preview=True
            )
            eval_ms.append((perf_counter() - started) * 1000.0)
            started = perf_counter()
            buffers = build_preview_buffers(faces)
            batch_ms.append((perf_counter() - started) * 1000.0)
            face_counts.append(len(faces))
            tri_counts.append(len(buffers["tri_positions"]) // 3)
        rows.append(
            {
                "width": width,
                "faces": face_counts[-1],
                "triangles": tri_counts[-1],
                "evaluator_ms_median": round(
                    statistics.median(eval_ms), 3
                ),
                "evaluator_ms_p95": round(
                    sorted(eval_ms)[int(0.95 * (len(eval_ms) - 1))], 3
                ),
                "batch_build_ms_median": round(
                    statistics.median(batch_ms), 3
                ),
                "batch_share_pct": round(
                    100.0
                    * statistics.median(batch_ms)
                    / max(1e-9, statistics.median(eval_ms)),
                    2,
                ),
                "depsgraph_ms": None,  # только живой Blender (MCP)
            }
        )
    return rows


def main():
    report = {
        "harness": "F0-lite gpu preview split (headless)",
        "fixtures": {
            "silhouette": _measure(*_silhouette_graph()),
            "fan_wall_24": _measure(*_fan_wall_graph()),
        },
        "note": (
            "batch-build — цена display-adapter поверх evaluator'а; "
            "mesh-write/depsgraph в GPU-режиме отсутствуют по построению"
        ),
    }
    print(json.dumps(report, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
