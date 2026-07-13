"""Дамп manual seam scope для offline-репро decal network (T043).

Запуск: Blender Text Editor -> открыть этот файл -> Run Script.
Объект должен быть активным; в Edit Mode берутся выбранные edges, в
Object Mode - все seam-marked edges. Результат: cftuv_seam_dump.json
рядом с .blend (или в temp каталоге Blender), путь печатается в консоль.

Дамп содержит всё, что потребляет decal_network: выбранные рёбра, их
вершины и нормали/центры смежных faces (материальная сторона фолда
восстанавливается по центрам). Приложите JSON к диалогу - сеть можно
переиграть вне Blender один в один.
"""

import json
import os

import bmesh
import bpy


def main():
    obj = bpy.context.edit_object or bpy.context.active_object
    if obj is None or obj.type != "MESH":
        raise RuntimeError("Нужен активный mesh-объект")

    owns_bmesh = False
    if obj.mode == "EDIT":
        bm = bmesh.from_edit_mesh(obj.data)
        edges = [edge for edge in bm.edges if edge.select]
    else:
        bm = bmesh.new()
        bm.from_mesh(obj.data)
        owns_bmesh = True
        edges = [edge for edge in bm.edges if edge.seam]

    settings = getattr(bpy.context.scene, "hotspotuv_settings", None)
    data = {
        "object": obj.name,
        "mode": obj.mode,
        "settings": {
            "decal_width_seam": getattr(settings, "decal_width_seam", None),
            "decal_offset": getattr(settings, "decal_offset", None),
            "decal_seam_network": getattr(settings, "decal_seam_network", None),
        },
        "edges": [
            {
                "index": edge.index,
                "seam": bool(edge.seam),
                "verts": [
                    {"index": vert.index, "co": list(vert.co)}
                    for vert in edge.verts
                ],
                "faces": [
                    {
                        "index": face.index,
                        "normal": list(face.normal),
                        "center": list(face.calc_center_median()),
                    }
                    for face in edge.link_faces
                ],
            }
            for edge in edges
        ],
    }
    if owns_bmesh:
        bm.free()

    base_dir = os.path.dirname(bpy.data.filepath) or bpy.app.tempdir
    out_path = os.path.join(base_dir, "cftuv_seam_dump.json")
    with open(out_path, "w", encoding="utf-8") as handle:
        json.dump(data, handle, ensure_ascii=False, indent=1)
    print(f"[CFTUV] seam dump: {out_path} ({len(data['edges'])} edges)")


main()
