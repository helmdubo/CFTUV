"""blender_gpu_drag: GPU_AFFINE_DRAG_V1 proofs, headless (the scene is never saved).

    blender -b scene.blend --python-exit-code 1 --python tools/blender_gpu_drag.py -- --root <tree> --native-path <dir> --case cover008 --workers 12 --out gpu.json

1. EQUALITY. After the cold button, at widths around the cold width (`--rels`), the model frame Python writes today (`envelope_width_preview_model.evaluate`, binary64 rounded to
   float32) against what the nodes show (the evaluated decal mesh, `evaluated_get` + `to_mesh`): the largest position and UV difference, and the vertices / loops whose difference is above
   `--tolerance` (a window edge decided differently in float32). The original mesh stays the base the whole time (its arrays hash before = after).
2. COST. `--events` widths swept like a drag: per event the main-thread cost of the frame and of the depsgraph evaluation that a redraw would do, with the nodes (one number into the socket)
   and with today's Python write (`foreach_set` + `mesh.update()`). Headless has no viewport: the GPU batch is not built here, the GUI probe (`tools/gpu_drag_gui_probe.py`) measures that.
`--coherent on` (LIVE_DRAG_COHERENT_V1): 1 and the ticks compare the coherent frame (every domain follows the hand, past its window) instead of the model frame.
`--overflow 0.02,0.05,...` (LIVE_DRAG_COHERENT_VERTEX_CLAMPED): only the metric "vertices past their limit": from the cold model, the frame shown at each relative width (the coherent one, and
   the plain model frame for reference) against a full exact run at that width (`tools/coherent_overflow.py`: area excess per patch over the exact area, faces turned over); the vertices the frame stopped at their limit are counted too.
3. REMOVAL. Every named removal path leaves no modifier, no attribute and no node group in the file: the cancel and the confirm (`finish_drag`), the handlers of save / load / undo
   (`envelope_width_modal`), the model drop, the stale leftover at the drag start; after the cancel the mesh is the base bit for bit.
Last line: GPU_DRAG_OK.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
import traceback
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import blender_drag_ticks as drag_ticks  # noqa: E402


class _OverflowOnly(Exception):
    """`--overflow`: the metric is the whole run."""


def _arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", required=True)
    parser.add_argument("--native-path", default="")
    parser.add_argument("--case", default="building")
    parser.add_argument("--workers", type=int, default=12)
    parser.add_argument("--rels", default="-0.10,-0.06,-0.03,-0.01,-0.002,0,0.002,0.01,0.03,0.06,0.10")
    parser.add_argument("--events", type=int, default=120)
    parser.add_argument("--span", type=float, default=0.05)
    parser.add_argument("--tolerance", type=float, default=1e-4)
    parser.add_argument("--kernel-backend", default="NATIVE")
    parser.add_argument("--overflow", default="", help="LIVE_DRAG_COHERENT_VERTEX_CLAMPED metric only: comma-separated relative widths (e.g. 0.02,0.05,0.1,0.2,0.4)")
    parser.add_argument("--overflow-share", type=float, default=0.01, help="area excess of a patch, as a share of its exact area, counted as spilling")
    parser.add_argument("--coherent", choices=("on", "off"), default="off")  # LIVE_DRAG_COHERENT_V1: the frame compared is the coherent one (every domain follows)
    parser.add_argument("--out", required=True)
    return parser.parse_args(sys.argv[sys.argv.index("--") + 1 :] if "--" in sys.argv else [])


def _evaluated(bpy, decal):
    import numpy as np

    depsgraph = bpy.context.evaluated_depsgraph_get()
    evaluated = decal.evaluated_get(depsgraph)
    mesh = evaluated.to_mesh()
    try:
        positions = np.empty(len(mesh.vertices) * 3, dtype=np.float32)
        mesh.vertices.foreach_get("co", positions)
        layer = mesh.uv_layers.get("UVMap")
        uvs = np.empty(len(layer.data) * 2, dtype=np.float32)
        layer.data.foreach_get("uv", uvs)
        names = sorted(item.name for item in mesh.attributes)
    finally:
        evaluated.to_mesh_clear()
    return positions, uvs, names


def _left_over(bpy, gpu) -> dict:
    return {
        "modifiers": sorted(item.name for item in bpy.data.objects if hasattr(item, "modifiers") and item.modifiers.get(gpu.MODIFIER_NAME) is not None),
        "meshes": sorted(mesh.name for mesh in bpy.data.meshes if any(mesh.attributes.get(name) is not None for name, *_rest in gpu.ATTRIBUTES)),
        "group": bpy.data.node_groups.get(gpu.GROUP_NAME) is not None,
    }


def _clean(left: dict) -> bool:
    return not left["modifiers"] and not left["meshes"] and not left["group"]


def _frame(controller, model, width):
    """The frame the preview shows: the model frame, or with LIVE_DRAG_COHERENT_V1 the coherent frame (extrapolated past the windows)."""

    import cftuv.envelope_width_mesh_preview as preview
    from cftuv.envelope_width_preview_model import coherent_frame, evaluate

    coherent = preview.coherent_now(controller)
    return evaluate(model, width) if coherent is None else coherent_frame(model, coherent, width)


def prove_equality(session, args, controller, decal, report) -> None:
    import numpy as np

    import cftuv.envelope_width_mesh_preview as preview

    bpy = session.bpy
    model = controller.width_model
    base = float(model.base_alpha)
    rows = []
    for rel in (float(item) for item in args.rels.split(",")):
        width = base * (1.0 + rel)
        frame = _frame(controller, model, width)
        state = preview.preview_mesh_now(controller, width)
        if state is None or controller.width_gpu_rig is None:
            raise RuntimeError(f"the nodes did not show width {width}: {preview.log_of(controller).refusals}")
        positions, uvs, names = _evaluated(bpy, decal)
        dp = np.abs(positions - frame.positions)
        du = np.abs(uvs - frame.uvs)
        scale = float(np.abs(frame.positions).max())
        # In units of the float32 spacing of each value (the rounding of one float32 operation is half of it): Python rounds binary64 once, the nodes round each float32 operation.
        p_ulps = dp / np.spacing(np.maximum(np.abs(positions), np.abs(frame.positions)).astype(np.float32))
        u_ulps = du / np.spacing(np.maximum(np.abs(uvs), np.abs(frame.uvs)).astype(np.float32))
        rows.append({
            "rel": rel, "width": width, "live_domains": frame.live_domains, "held_domains": frame.held_domains, "extrapolated": frame.extrapolated, "estimated": frame.estimated, "summary_live": state.live, "summary_held": state.held,
            "max_position_diff": float(dp.max()), "max_uv_diff": float(du.max()), "position_ulp": float(np.spacing(np.float32(scale))),
            "max_position_ulps": float(p_ulps.max()), "max_uv_ulps": float(u_ulps.max()), "uv_abs_max": float(np.abs(frame.uvs).max()),
            "vertices_above_4ulp": int((p_ulps.reshape(-1, 3).max(axis=1) > 4).sum()), "loops_above_4ulp": int((u_ulps.reshape(-1, 2).max(axis=1) > 4).sum()),
            "vertices_above_tolerance": int((dp.reshape(-1, 3).max(axis=1) > args.tolerance).sum()), "loops_above_tolerance": int((du.reshape(-1, 2).max(axis=1) > args.tolerance).sum()),
            "moved_max": float(np.abs(frame.positions - drag_ticks_base(report)).max()) if "base_positions" in report else None,
            "helper_attributes_in_evaluated_mesh": [name for name in names if name.startswith(".cftuv_gn_")],
        })
    report["equality"] = rows
    report["equality_max"] = {"position": max(row["max_position_diff"] for row in rows), "uv": max(row["max_uv_diff"] for row in rows),
                              "position_ulps": max(row["max_position_ulps"] for row in rows), "uv_ulps": max(row["max_uv_ulps"] for row in rows),
                              "vertices_above_4ulp": sum(row["vertices_above_4ulp"] for row in rows), "loops_above_4ulp": sum(row["loops_above_4ulp"] for row in rows),
                              "vertices_above_tolerance": sum(row["vertices_above_tolerance"] for row in rows), "loops_above_tolerance": sum(row["loops_above_tolerance"] for row in rows)}


def drag_ticks_base(report):
    import numpy as np

    return np.asarray(report["base_positions"], dtype=np.float32)


def measure_cost(session, args, controller, report) -> None:
    import cftuv.envelope_width_mesh_preview as preview

    bpy = session.bpy
    base = float(controller.width_model.base_alpha)
    widths = [base * (1.0 + args.span * ((index % 40) / 20.0 - 1.0)) for index in range(args.events)]
    out = {}
    for mode in ("nodes", "python"):
        controller.width_gpu_drag = mode == "nodes"
        if mode == "python":
            preview.retire_gpu(controller, "TOOL_SWITCH")
        frame_costs, eval_costs, totals = [], [], []
        for width in widths:
            started = time.perf_counter()
            state = preview.preview_mesh_now(controller, width)
            framed = time.perf_counter()
            bpy.context.view_layer.update()
            finished = time.perf_counter()
            if state is None:
                raise RuntimeError(f"{mode}: no frame at {width}")
            frame_costs.append(framed - started)
            eval_costs.append(finished - framed)
            totals.append(finished - started)
        out[mode] = {"frame": drag_ticks.stats(frame_costs), "evaluate": drag_ticks.stats(eval_costs), "total": drag_ticks.stats(totals)}
    controller.width_gpu_drag = True
    report["cost"] = out


def prove_ticks(session, controller, decal, report, count: int = 3) -> None:
    """Exact drag ticks laid while the nodes stand: after each one the evaluated mesh is the tick itself (the nodes were set again at zero shift), and a frame after it is the NEW model's frame."""

    import numpy as np

    import cftuv.envelope_width_mesh_preview as preview

    bpy = session.bpy
    laps = drag_ticks.Laps()
    rows = []
    for index in range(count):
        base = float(controller.width_model.base_alpha)
        preview.preview_mesh_now(controller, base * 1.004)  # the hand is away from the base: the nodes move the mesh
        tick = session.step(base * 1.004, lean=True, laps=laps, incremental=True)
        model = controller.width_model
        positions, uvs, _names = _evaluated(bpy, decal)
        at_tick = _frame(controller, model, model.base_alpha)
        frame_width = model.base_alpha * 1.003
        preview.preview_mesh_now(controller, frame_width)
        moved_positions, moved_uvs, _names = _evaluated(bpy, decal)
        frame = _frame(controller, model, frame_width)
        rows.append({
            "tick": tick["apply_outcome"], "plan": (tick["plan"] or {}).get("mode"), "installs": preview.log_of(controller).gpu_installs,
            "after_tick_position_diff": float(np.abs(positions - at_tick.positions).max()), "after_tick_uv_diff": float(np.abs(uvs - at_tick.uvs).max()),
            "frame_position_diff": float(np.abs(moved_positions - frame.positions).max()), "frame_uv_diff": float(np.abs(moved_uvs - frame.uvs).max()),
        })
    report["ticks_under_nodes"] = rows


def prove_removal(session, controller, decal, base_hash, report) -> None:
    import cftuv.envelope_width_gpu_drag as gpu
    import cftuv.envelope_width_mesh_preview as preview
    import cftuv.envelope_width_modal as modal
    from cftuv.envelope_width_live import begin_gpu_drag

    bpy = session.bpy
    width = float(controller.width_model.base_alpha) * 1.02
    paths = {}

    def show():
        controller.width_drag_active = True
        controller.width_gpu_drag = True
        if preview.preview_mesh_now(controller, width) is None or controller.width_gpu_rig is None:
            raise RuntimeError("the nodes were not installed")
        bpy.context.view_layer.update()
        if _clean(_left_over(bpy, gpu)):
            raise RuntimeError("installed nodes not seen")

    show()
    preview.finish_drag(controller, restore=True)
    paths[gpu.REMOVED_AT_CANCEL] = {"left": _left_over(bpy, gpu), "mesh_is_base": drag_ticks.mesh_arrays_hash(decal.data) == base_hash}
    show()
    preview.finish_drag(controller, restore=False)
    paths[gpu.REMOVED_AT_CONFIRM] = {"left": _left_over(bpy, gpu)}
    for name, handler in ((gpu.REMOVED_ON_SAVE, modal._before_save), (gpu.REMOVED_ON_LOAD, modal._before_load), (gpu.REMOVED_ON_HISTORY, modal._before_history)):
        show()
        handler(bpy.context.scene)
        paths[name] = {"left": _left_over(bpy, gpu), "rig_forgotten": controller.width_gpu_rig is None}
    show()
    controller.width_drag_active = False
    controller.width_gpu_rig = None  # a lost rig (as after a crash of the session): only the drag start can find it
    begin_gpu_drag(controller, session.mesh_settings)
    paths[gpu.REMOVED_STALE] = {"left": _left_over(bpy, gpu)}
    show()
    preview.drop_model(controller, "GPU_DRAG_TOOL_DROP")
    paths[gpu.REMOVED_ON_MODEL_DROP] = {"left": _left_over(bpy, gpu)}
    controller.width_drag_active = False
    report["removal"] = paths
    report["removal_all_clean"] = all(_clean(item["left"]) for item in paths.values())
    report["removal_log"] = dict(preview.log_of(controller).gpu_removed)


def measure_overflow(session, args, controller, report) -> None:
    """The metric "vertices past their limit" (LIVE_DRAG_COHERENT_VERTEX_CLAMPED) of the shown frames against exact runs at the same widths."""

    import coherent_overflow

    import cftuv.envelope_width_mesh_preview as preview
    from dataclasses import replace

    from cftuv.envelope_width_preview_model import build_model, coherent_frame, coherent_slopes, evaluate

    model = controller.width_model
    base = model.base
    coherent = coherent_slopes(model)
    # "Before": the same frame with the vertex limits taken away (as LIVE_DRAG_COHERENT_V1 without VERTEX_ALPHA_LIMITS_V1).
    bare = build_model(replace(base, domains=tuple(replace(item, limits=None, floors=None) for item in base.domains), cache={}))
    unclamped = coherent_slopes(bare)
    widths = [base.alpha * (1.0 + float(item)) for item in args.overflow.split(",") if item.strip()]
    frames = {width: (coherent_frame(model, coherent, width), evaluate(model, width), coherent_frame(bare, unclamped, width)) for width in widths}
    laps = drag_ticks.Laps()
    rows = []
    for width in widths:
        session.step(width, lean=True, laps=laps)
        exact = controller.width_displayed
        if exact is None or abs(exact.alpha - width) > 1e-12:
            rows.append({"width": width, "failure": "no exact sample at the width"})
            continue
        shown, plain, free = frames[width]
        found = coherent_overflow.overflow(shown.positions, base, exact, args.overflow_share)
        plain_found = coherent_overflow.overflow(plain.positions, base, exact, args.overflow_share)
        free_found = coherent_overflow.overflow(free.positions, base, exact, args.overflow_share)
        rows.append({"width": width, "rel": width / base.alpha - 1.0, "coherent": found, "plain": plain_found, "unclamped": free_found, "coherent_clamped": shown.clamped,
                     "coherent_extrapolated_domains": shown.extrapolated, "plain_held_domains": plain.held_domains})
    report["overflow"] = {"base_width": base.alpha, "vertex_limits": coherent.vertex_cap is not None, "rows": rows,
                          "log_clamped_max": preview.log_of(controller).coherent_clamped_max}


def main() -> int:
    args = _arguments()
    args.grade, args.ticks, args.confirms = "commit", 0, 0
    report: dict = {"schema": "cftuv.gpu-drag.v1", "args": vars(args), "failure": None}
    try:
        session = drag_ticks.Session(args)
        report["native_build_id"] = session.native["build_id"]
        report["cold_press_s"] = round(session.cold_press(), 4)
        if session.bpy.context.mode != "OBJECT":
            session.bpy.ops.object.mode_set(mode="OBJECT")
        import numpy as np

        import cftuv.envelope_width_mesh_preview as preview

        controller = session.controller
        if not preview.ensure_model(controller):
            raise RuntimeError(f"no model: {controller.width_model_refusal}")
        decal = session.bpy.data.objects[session.mesh_module.decal_object_name(session.obj.name)]
        mesh = decal.data
        report["mesh"] = {"vertices": len(mesh.vertices), "loops": len(mesh.loops), "polygons": len(mesh.polygons), "domains": controller.width_model.domain_count,
                          "modelled": controller.width_model.modelled_domains}
        base_hash = drag_ticks.mesh_arrays_hash(mesh)
        base_digest = session.mesh_module.mesh_content_digest(mesh)
        positions = np.empty(len(mesh.vertices) * 3, dtype=np.float32)
        mesh.vertices.foreach_get("co", positions)
        report["base_positions"] = positions.tolist()
        controller.width_drag_active = True
        if args.overflow:
            measure_overflow(session, args, controller, report)
            raise _OverflowOnly
        controller.width_gpu_drag = True
        controller.width_coherent_drag = args.coherent == "on"
        prove_equality(session, args, controller, decal, report)
        report["original_mesh_stays_base"] = drag_ticks.mesh_arrays_hash(mesh) == base_hash
        measure_cost(session, args, controller, report)
        preview.finish_drag(controller, restore=True)
        report["after_cancel_is_base"] = drag_ticks.mesh_arrays_hash(mesh) == base_hash and session.mesh_module.mesh_content_digest(mesh) == base_digest
        controller.width_drag_active = True
        controller.width_gpu_drag = True
        controller.width_coherent_drag = args.coherent == "on"
        prove_ticks(session, controller, decal, report)
        preview.finish_drag(controller, restore=True)  # the cancel after exact ticks: the mesh is what was shown before the first of them
        report["after_ticks_cancel_is_base"] = drag_ticks.mesh_arrays_hash(mesh) == base_hash
        prove_removal(session, controller, decal, base_hash, report)
        del report["base_positions"]
    except _OverflowOnly:
        report.pop("base_positions", None)
    except Exception:  # noqa: BLE001 - the traceback is the report
        report["failure"] = traceback.format_exc()
        report.pop("base_positions", None)
        print(report["failure"])
    try:
        from cftuv.envelope_domain_pool import shutdown_domain_pool

        shutdown_domain_pool()
    except Exception as exc:  # noqa: BLE001
        print("pool shutdown:", type(exc).__name__, exc)
    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    Path(args.out).write_text(json.dumps(report, ensure_ascii=False, indent=1, sort_keys=True, default=str) + "\n", encoding="utf-8")
    if args.overflow:
        ok = report["failure"] is None and "overflow" in report
        print("GPU_DRAG_OVERFLOW_OK" if ok else "GPU_DRAG_OVERFLOW_FAILED")
        return 0 if ok else 1
    ticks_ok = all(max(row["after_tick_position_diff"], row["frame_position_diff"]) < 1e-4 and max(row["after_tick_uv_diff"], row["frame_uv_diff"]) < 1e-3 for row in report.get("ticks_under_nodes", []))
    ok = (report["failure"] is None and report.get("removal_all_clean") and report.get("original_mesh_stays_base") and report.get("after_cancel_is_base")
          and report.get("after_ticks_cancel_is_base") and ticks_ok)
    if report["failure"] is None:
        print(f"GPU_DRAG equality max position {report['equality_max']['position']:.3g} uv {report['equality_max']['uv']:.3g} cost {json.dumps(report['cost'])}")
    print("GPU_DRAG_OK" if ok else "GPU_DRAG_FAILED")
    return 0 if ok else 1


if __name__ == "__main__":
    code = main()
    if code:
        raise SystemExit(code)
