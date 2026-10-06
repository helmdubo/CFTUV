"""Полевой корпус против предфильтра «контактов нет»: на КАЖДОЙ паре «источник, отрезок границы», которую видит кнопка, фильтр сверяется с точным путём.

    blender -b E:/testscene.blend --python-exit-code 1 --python field_scan.py -- --root <дерево> --out <json> \
        --cases mesh:alpha:density:stretch,...

Кнопка идёт последовательно (`envelope_debug_workers = 0`: счёт в этом процессе, иначе обёртка воркерам не видна), ничего не сохраняется.
Обёртка `boundary._contact_candidates` на каждой паре с кадром источника считает ТОЧНЫЙ ответ (`contact_candidates_native` без предфильтра)
и вердикт `SourceContactFrame.excludes`; пишутся числа пар, отвергнутых, пустых, пустых без доказательства и ЛОЖНЫХ отвержений
(отвергнута пара, у которой точный ответ не пуст). Ложных отвержений обязано быть ноль, расхождение ответа обёртки с точным - ноль.
Код возврата 1 при любом ненулевом.
"""

import argparse
import json
import sys
import traceback
from pathlib import Path

import bmesh
import bpy


def _arguments():
    tail = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", required=True)
    parser.add_argument("--out", required=True)
    parser.add_argument("--cases", required=True)
    return parser.parse_args(tail)


def _load_tree(root):
    installed = sys.modules.get("cftuv")
    if installed is not None:
        try:
            installed.unregister()
        except Exception as exc:
            print("unregister failed", exc)
    for name in tuple(sys.modules):
        if name in {"cftuv", "cftuv_envelope"} or name.startswith(("cftuv.", "cftuv_envelope.")):
            del sys.modules[name]
    for path in (root / "kernel" / "src", root):
        if str(path) in sys.path:
            sys.path.remove(str(path))
        sys.path.insert(0, str(path))
    import cftuv
    import cftuv_envelope

    assert Path(cftuv.__file__).resolve().parent == (root / "cftuv").resolve(), cftuv.__file__
    assert Path(cftuv_envelope.__file__).resolve().parent == (root / "kernel" / "src" / "cftuv_envelope").resolve()
    cftuv.register()


STATS = {}


def _install_scan():
    from cftuv_envelope.reference import boundary

    real = boundary._contact_candidates

    def scanned(context, source, segment, frame=None):
        out = real(context, source, segment, frame)
        if frame is None:
            return out
        exact = boundary.contact_candidates_native(context, source, segment)
        proved = frame.excludes(segment)
        STATS["pairs"] = STATS.get("pairs", 0) + 1
        STATS["empty"] = STATS.get("empty", 0) + (not exact)
        STATS["rejected"] = STATS.get("rejected", 0) + bool(proved)
        STATS["empty_not_rejected"] = STATS.get("empty_not_rejected", 0) + ((not exact) and not proved)
        STATS["false_rejections"] = STATS.get("false_rejections", 0) + bool(proved and exact)
        STATS["answer_mismatch"] = STATS.get("answer_mismatch", 0) + (len(out) != len(exact))
        return out

    boundary._contact_candidates = scanned


def _press(obj, alpha):
    settings = bpy.context.scene.hotspotuv_settings
    bpy.context.view_layer.objects.active = obj
    if bpy.context.mode != "OBJECT":
        bpy.ops.object.mode_set(mode="OBJECT")
    for other in bpy.context.selected_objects:
        other.select_set(False)
    obj.select_set(True)
    bpy.ops.object.mode_set(mode="EDIT")
    bpy.context.tool_settings.mesh_select_mode = (False, True, False)
    bm = bmesh.from_edit_mesh(obj.data)
    bm.edges.ensure_lookup_table()
    for edge in bm.edges:
        edge.select_set(bool(edge.seam))
    bmesh.update_edit_mesh(obj.data)
    settings.envelope_debug_alpha = alpha
    try:
        status = bpy.ops.hotspotuv.build_envelope_decal_mesh()
    except RuntimeError as exc:
        status = {"ERROR: " + str(exc).strip()[:120]}
    if bpy.context.mode != "OBJECT":
        bpy.ops.object.mode_set(mode="OBJECT")
    return sorted(status)


def _case(spec):
    mesh_name, alpha_text, density, stretch = spec.split(":")
    settings = bpy.context.scene.hotspotuv_settings
    settings.envelope_debug_engine = "QUEUE"
    settings.envelope_debug_workers = 0
    settings.envelope_debug_fan_density = density
    settings.envelope_debug_max_stretch = int(stretch)
    from cftuv.envelope_debug_session import EnvelopeDebugSessionController, WINDOW_MANAGER_SESSION_ATTRIBUTE

    controller = getattr(bpy.context.window_manager, WINDOW_MANAGER_SESSION_ATTRIBUTE, None)
    if not isinstance(controller, EnvelopeDebugSessionController):
        controller = EnvelopeDebugSessionController()
        setattr(bpy.context.window_manager, WINDOW_MANAGER_SESSION_ATTRIBUTE, controller)
    controller.clear()
    STATS.clear()
    status = _press(bpy.data.objects[mesh_name], float(alpha_text))
    return {"case": spec, "operator": status, **dict(sorted(STATS.items()))}


def main():
    args = _arguments()
    _load_tree(Path(args.root).resolve())
    _install_scan()
    rows = []
    for spec in args.cases.split(","):
        try:
            row = _case(spec)
        except Exception:
            row = {"case": spec, "failure": traceback.format_exc()[-400:]}
        rows.append(row)
        print("SCAN_ROW", json.dumps(row, sort_keys=True), flush=True)
    totals = {key: sum(row.get(key, 0) for row in rows) for key in ("pairs", "empty", "rejected", "empty_not_rejected", "false_rejections", "answer_mismatch")}
    failed = bool(totals["false_rejections"] or totals["answer_mismatch"] or any("failure" in row for row in rows))
    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    Path(args.out).write_text(json.dumps({"totals": totals, "cases": rows}, ensure_ascii=False, indent=1, sort_keys=True) + "\n", encoding="utf-8")
    print("SCAN_TOTALS", json.dumps(totals, sort_keys=True), "FAILED" if failed else "CLEAN", flush=True)
    sys.stdout.flush()
    if failed:
        raise SystemExit(1)


main()
