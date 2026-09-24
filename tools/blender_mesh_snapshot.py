"""Слепок меша для фикстур ядра: активный объект + выделенные рёбра → JSON.

Зачем. Полевые дефекты (U-цепь, фасочный пояс) нельзя закрепить фикстурой
без топологии, а .blend в репозиторий не кладётся. Слепок несёт ровно то,
что видит конвейер: координаты object-space (binary64 без округления —
json round-trip точен), топологию, флаги seam/sharp/selected и матрицу
объекта. Материалы, UV и имена сцены не выгружаются намеренно.

Как пользоваться владельцу:
1. Выделить рёбра интересующего маршрута в Edit Mode, вернуться в Object Mode.
2. Запустить скрипт в Blender (Text Editor → Run, либо
   `blender --background file.blend --python tools/blender_mesh_snapshot.py`).
3. Файл пишется рядом с .blend: `<имя_объекта>_snapshot.json`; путь — в консоли.

Слепок двухслойный: RAW — авторская сетка с выделением; EVALUATED —
то, что видит экспортёр (модификаторы + триангуляция). Если топологии
совпадают (нет генерирующих модификаторов), `raw_matches_evaluated=true`
и выделение переносимо на evaluated по индексам.
"""

from __future__ import annotations

import json
import os

import bpy

SCHEMA = "cftuv.mesh_snapshot.v1"


def _vertices(mesh) -> list:
    return [[v.co.x, v.co.y, v.co.z] for v in mesh.vertices]


def _edges(mesh, with_flags: bool) -> list:
    rows = []
    for edge in mesh.edges:
        row = [edge.vertices[0], edge.vertices[1]]
        if with_flags:
            row.append(
                {
                    "seam": int(edge.use_seam),
                    "sharp": int(edge.use_edge_sharp),
                    "selected": int(edge.select),
                }
            )
        rows.append(row)
    return rows


def _faces(mesh) -> list:
    return [list(polygon.vertices) for polygon in mesh.polygons]


def _loop_triangles(mesh) -> list:
    mesh.calc_loop_triangles()
    return [
        [tri.vertices[0], tri.vertices[1], tri.vertices[2], tri.polygon_index]
        for tri in mesh.loop_triangles
    ]


def snapshot(obj) -> dict:
    raw = obj.data
    record = {
        "schema": SCHEMA,
        "blender_version": bpy.app.version_string,
        "object_name": obj.name,
        "matrix_world": [list(row) for row in obj.matrix_world],
        "modifiers": [
            {"name": m.name, "type": m.type, "show_viewport": m.show_viewport}
            for m in obj.modifiers
        ],
        "raw": {
            "vertices": _vertices(raw),
            "edges": _edges(raw, with_flags=True),
            "faces": _faces(raw),
            "selected_edges": [e.index for e in raw.edges if e.select],
            "selected_vertices": [v.index for v in raw.vertices if v.select],
        },
    }
    depsgraph = bpy.context.evaluated_depsgraph_get()
    evaluated = obj.evaluated_get(depsgraph)
    mesh_eval = evaluated.to_mesh()
    try:
        record["evaluated"] = {
            "vertices": _vertices(mesh_eval),
            "edges": _edges(mesh_eval, with_flags=False),
            "faces": _faces(mesh_eval),
            "loop_triangles": _loop_triangles(mesh_eval),
        }
        record["raw_matches_evaluated"] = (
            len(raw.vertices) == len(mesh_eval.vertices)
            and len(raw.edges) == len(mesh_eval.edges)
            and len(raw.polygons) == len(mesh_eval.polygons)
        )
    finally:
        evaluated.to_mesh_clear()
    return record


def main() -> str:
    obj = bpy.context.active_object
    if obj is None or obj.type != "MESH":
        raise RuntimeError("Активный объект не меш: выберите объект слепка")
    if bpy.context.mode != "OBJECT":
        bpy.ops.object.mode_set(mode="OBJECT")
    record = snapshot(obj)
    base = bpy.path.abspath("//") or os.getcwd()
    safe_name = "".join(
        ch if ch.isalnum() or ch in "-_." else "_" for ch in obj.name
    )
    path = os.path.join(base, f"{safe_name}_snapshot.json")
    with open(path, "w", encoding="utf-8") as handle:
        json.dump(record, handle, ensure_ascii=False)
    selected = len(record["raw"]["selected_edges"])
    print(
        f"CFTUV snapshot: {path}\n"
        f"  vertices={len(record['raw']['vertices'])}"
        f" edges={len(record['raw']['edges'])}"
        f" faces={len(record['raw']['faces'])}"
        f" selected_edges={selected}"
        f" raw_matches_evaluated={record['raw_matches_evaluated']}"
    )
    if selected == 0:
        print("  ВНИМАНИЕ: выделенных рёбер нет — маршрут не размечен")
    return path


if __name__ == "__main__":
    main()
