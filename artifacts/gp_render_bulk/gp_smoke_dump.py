"""Дампы GP на маленьких сценах смоков: Blender 4.3 и 4.5, до/после изменения.

    blender --background [--factory-startup] --python-exit-code 1 \
        --python gp_smoke_dump.py -- --worktree <дерево> --out <каталог> \
        --tag <метка> --variant <имя>

Один вариант на процесс Blender: материалы `CFTUV_EnvelopeDebug_*` общие по
имени, и соседний вариант иначе делил бы их с предыдущим.

Варианты:
  topology       кнопка топологии на двух патчах (слои ENV_0x/1x, ENV_LABELS);
  queue          QUEUE, alpha 0.25, два патча (слои очереди и владельцев);
  queue_alpha    то же + смена alpha: лёгкая перерисовка `attach` + `commit`;
  multi_seam     QUEUE на шве из двух коллинеарных рёбер;
  legacy_viz     `debug_analysis` на кубе со швами: `create_visualization` и
                 `create_frontier_visualization` (кадры реплея, заливки);
  legacy_exact   движок LEGACY на двух патчах (Blender 4.5: нужен sympy): точные
                 пути, петли, области, точки, метки и диагностики сцен;
  control_mutated  отрицательный контроль: `queue` + сдвиг ОДНОЙ координаты на
                 2e-7 (порядка одного ULP float32) — дамп обязан отличаться.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

import bpy


def _arguments():
    values = sys.argv[sys.argv.index("--") + 1 :]
    parsed = {}
    for index in range(0, len(values), 2):
        parsed[values[index].lstrip("-")] = values[index + 1]
    return parsed


def _cube_with_seams():
    bpy.ops.mesh.primitive_cube_add(size=2.0)
    obj = bpy.context.active_object
    obj.name = "SeamCube"
    bpy.ops.object.mode_set(mode="EDIT")
    bpy.ops.mesh.select_all(action="SELECT")
    bpy.ops.mesh.subdivide(number_cuts=1)
    bpy.ops.object.mode_set(mode="OBJECT")
    for edge in obj.data.edges:
        a = obj.data.vertices[edge.vertices[0]].co
        b = obj.data.vertices[edge.vertices[1]].co
        fixed = [
            axis
            for axis in range(3)
            if abs(abs(a[axis]) - 1.0) < 1e-6 and abs(a[axis] - b[axis]) < 1e-6
        ]
        edge.use_seam = len(fixed) >= 2
    return obj


def main():
    args = _arguments()
    worktree = Path(args["worktree"]).resolve()
    out = Path(args["out"])
    out.mkdir(parents=True, exist_ok=True)
    variant = args["variant"]
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    # Строители сцен смоков берутся из ДЕРЕВА ПОД ПРОВЕРКОЙ; их импорт чистит
    # `cftuv*` из sys.modules, поэтому он идёт ДО регистрации аддона.
    sys.path.insert(0, str(worktree / "tests" / "blender"))
    import test_envelope_debug_bridge as bridge

    import gp_dump
    from gp_probe_common import bootstrap

    bootstrap(worktree)
    bridge._reset_scene()
    settings = bpy.context.scene.hotspotuv_settings
    report = {"version": bpy.app.version_string, "variant": variant}

    def dump(name, gp_object, source_name):
        report[name] = gp_dump.write_dump(
            gp_object, out / f"{args['tag']}_{name}_dump.json", source_name
        )
        print(
            f"[SMOKE] {bpy.app.version_string} {name}: "
            f"strokes={report[name]['strokes']} points={report[name]['points']} "
            f"sha256={report[name]['sha256']}"
        )

    if variant == "legacy_viz":
        obj = _cube_with_seams()
        bpy.context.view_layer.objects.active = obj
        assert bpy.ops.hotspotuv.debug_analysis() == {"FINISHED"}
        gp_name = "CFTUV_Debug_" + obj.name
        dump("legacy_viz", bpy.data.objects[gp_name], None)
    else:
        if variant == "multi_seam":
            obj, _seams = bridge._build_two_patch_multi_edge_seam()
        else:
            obj = bridge._build_two_patch_seam()
        if variant == "topology":
            assert (
                bpy.ops.hotspotuv.build_envelope_topology_debug() == {"FINISHED"}
            )
        else:
            settings.envelope_debug_engine = (
                "LEGACY" if variant == "legacy_exact" else "QUEUE"
            )
            settings.envelope_debug_alpha = 0.25
            settings.envelope_debug_workers = 0
            assert (
                bpy.ops.hotspotuv.build_exact_reference_envelope_debug()
                == {"FINISHED"}
            )
        gp_object = bpy.data.objects["CFTUV_DEBUG_Envelope_" + obj.name]
        if variant != "control_mutated":
            dump(variant, gp_object, obj.name)
        if variant == "control_mutated":
            for layer in gp_object.data.layers:
                if layer.frames and len(layer.frames[0].drawing.strokes):
                    position = layer.frames[0].drawing.attributes["position"]
                    x, y, z = position.data[0].vector
                    position.data[0].vector = (x + 2e-7, y, z)
                    break
            dump("control_mutated", gp_object, obj.name)
        if variant == "queue_alpha":
            settings.envelope_debug_alpha = 0.5
            gp_object = bpy.data.objects["CFTUV_DEBUG_Envelope_" + obj.name]
            dump("queue_alpha_redraw", gp_object, obj.name)

    (out / f"{args['tag']}_{variant}_report.json").write_text(
        json.dumps(report, indent=2, ensure_ascii=False), encoding="utf-8"
    )
    print("GP_SMOKE_DUMP_OK")


main()
