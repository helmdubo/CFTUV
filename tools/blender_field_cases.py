"""Полевые случаи настоящей кнопки в Blender без окна: запись для `artifacts/materialize_sweep/field_judge.py` (ничего не сохраняется).

    blender -b <сцена.blend> --python-exit-code 1 --python tools/blender_field_cases.py -- --out <json> --cases меш:alpha:плотность:растяжение,... \
        [--root <дерево>] [--backend PYTHON|DEFAULT] [--workers N] [--geom-dir DIR] [--wheel-site DIR]

--backend PYTHON : свойство сцены `kernel_backend` (главный переключатель; у деревьев до единого переключателя ещё и `skeleton_backend`) заказано `PYTHON` - все стадии эталон.
--backend DEFAULT: свойства СНЯТЫ (`property_unset`), и действует умолчание продукта ДЕРЕВА.
--wheel-site     : каталог с колесом `cftuv_native`; ставится первым в `sys.path` (иначе Blender берёт модуль из своих пользовательских скриптов).
Запись строки на случай: оператор, статус, исходы доменов, V/E/F, дайджест меша (`mesh_content_digest`), секунды кнопки, а с `--geom-dir` - точная геометрия (позиции,
UV по углам, грани, рёбра, швы) gzip-JSON: два прогона сравниваются побитово (`geometry_sha256`). Запуск - с `PYTHONSAFEPATH=1`.
"""
import argparse
import gzip
import hashlib
import json
import sys
import time
import traceback
from collections import Counter
from pathlib import Path

import bmesh
import bpy


def _arguments():
    tail = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", default=str(Path(__file__).resolve().parents[1]))
    parser.add_argument("--out", required=True)
    parser.add_argument("--cases", required=True)
    parser.add_argument("--backend", default="DEFAULT", choices=("DEFAULT", "PYTHON"))
    parser.add_argument("--workers", type=int, default=0)
    parser.add_argument("--geom-dir", default="")
    parser.add_argument("--wheel-site", default="")
    return parser.parse_args(tail)


_ARGS = None
_SETTING_REPORT = {}


def _load_tree(root):
    if _ARGS.wheel_site:
        sys.modules.pop("cftuv_native", None)
        sys.path.insert(0, str(Path(_ARGS.wheel_site).resolve()))
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
    mesh_settings = bpy.context.scene.hotspotuv_decal_mesh
    for name in ("kernel_backend", "skeleton_backend"):
        present = hasattr(mesh_settings, name)
        entry = {"present": present}
        if present:
            entry["was_set"] = bool(mesh_settings.is_property_set(name))
            entry["was_value"] = str(getattr(mesh_settings, name))
            if _ARGS.backend == "PYTHON":
                setattr(mesh_settings, name, "PYTHON")
            else:
                try:
                    mesh_settings.property_unset(name)
                except Exception as exc:
                    entry["unset_error"] = str(exc)
            entry["now_value"] = str(getattr(mesh_settings, name))
        _SETTING_REPORT[name] = entry
    print("SETTINGS", json.dumps(_SETTING_REPORT, sort_keys=True), flush=True)
    try:
        # Нативное ядро видно только через `backend.py` (правило архитектуры): путь модуля берётся из уже загруженных, не импортом.
        from cftuv_envelope.backend import native_status
        status = native_status().as_record()  # первый вызов импортирует расширение внутри `backend.py`
        print("NATIVE_STATUS", json.dumps(status), flush=True)
        print("NATIVE_MODULE", getattr(sys.modules.get("cftuv_native"), "__file__", None), flush=True)
    except Exception as exc:
        print("NATIVE_STATUS_UNAVAILABLE", type(exc).__name__, exc, flush=True)


CAPTURED = {}


def _install_capture():
    from cftuv import envelope_production_export, envelope_production_operator

    original_run = envelope_production_export.run_production

    def run_production(*a, **k):
        run = original_run(*a, **k)
        CAPTURED["run"] = run
        return run

    envelope_production_export.run_production = run_production
    original_write = envelope_production_operator.write_decal_object

    def write_decal_object(*a, **k):
        receipt = original_write(*a, **k)
        CAPTURED["receipt"] = receipt
        return receipt

    envelope_production_operator.write_decal_object = write_decal_object


def _reset_session():
    from cftuv.envelope_debug_session import EnvelopeDebugSessionController, WINDOW_MANAGER_SESSION_ATTRIBUTE

    controller = getattr(bpy.context.window_manager, WINDOW_MANAGER_SESSION_ATTRIBUTE, None)
    if not isinstance(controller, EnvelopeDebugSessionController):
        controller = EnvelopeDebugSessionController()
        setattr(bpy.context.window_manager, WINDOW_MANAGER_SESSION_ATTRIBUTE, controller)
    controller.clear()


def _select_seams(obj):
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


def _press(obj, alpha):
    CAPTURED.pop("run", None)
    CAPTURED.pop("receipt", None)
    settings = bpy.context.scene.hotspotuv_settings
    _select_seams(obj)
    settings.envelope_debug_alpha = alpha
    started = time.perf_counter()
    op_error = None
    try:
        status = bpy.ops.hotspotuv.build_envelope_decal_mesh()
    except RuntimeError as exc:
        status = {"ERROR"}
        op_error = str(exc).strip()[:200]
    seconds = time.perf_counter() - started
    if bpy.context.mode != "OBJECT":
        bpy.ops.object.mode_set(mode="OBJECT")
    return status, op_error, seconds


def _geometry(me):
    uv = me.uv_layers.active
    return {
        "positions": [[repr(c) for c in v.co] for v in me.vertices],
        "edges": [list(e.vertices) for e in me.edges],
        "seams": [bool(e.use_seam) for e in me.edges],
        "faces": [list(p.vertices) for p in me.polygons],
        "loops": [[p.loop_start, p.loop_total] for p in me.polygons],
        "uv": [[repr(c) for c in d.uv] for d in uv.data] if uv is not None else None,
        # атрибуты граней (`cftuv_domain`, `cftuv_owner`) входят в `mesh_content_digest`: без них дамп не объяснил бы расхождение дайджеста
        "attributes": {name: ([item.value for item in me.attributes[name].data] if name in me.attributes else None) for name in ("cftuv_domain", "cftuv_owner")},
    }


def _case(spec):
    mesh_name, alpha_text, density, stretch = spec.split(":")
    alpha = float(alpha_text)
    obj = bpy.data.objects[mesh_name]
    settings = bpy.context.scene.hotspotuv_settings
    settings.envelope_debug_engine = "QUEUE"
    settings.envelope_debug_workers = _ARGS.workers
    settings.envelope_debug_fan_density = density
    settings.envelope_debug_max_stretch = int(stretch)
    _reset_session()
    status, op_error, seconds = _press(obj, alpha)
    row = {
        "case": spec,
        "operator": sorted(status),
        "op_error": op_error,
        "button_seconds": round(seconds, 2),
        "status": str(bpy.context.scene.hotspotuv_decal_mesh.status),
        "src_faces": len(obj.data.polygons),
    }
    run = CAPTURED.get("run")
    outcomes = Counter()
    refused = []
    if run is not None:
        for item in run.results:
            name = str(item.outcome).split(".")[-1]
            outcomes[name] += 1
            if name != "MATERIALIZED":
                refused.append([item.patch_id, name, str(item.detail)[:160]])
        row["domain_seconds"] = round(sum(item.seconds for item in run.results), 3)
        row["backend_requested"] = str(getattr(run, "kernel_backend", getattr(run, "backend", None)))
        row["skeleton_backend_requested"] = str(getattr(run, "skeleton_backend", "n/a"))
        row["embedding_backend_requested"] = str(getattr(run, "embedding_backend", "n/a"))
        row["backend_ran"] = dict(sorted(Counter(str(getattr(getattr(item, "backend_record", None), "ran", "none")) for item in run.results).items()))
        record_attrs = {}
        for item in run.results:
            record = getattr(item, "backend_record", None)
            if record is None:
                continue
            for key in ("native_calls", "python_calls", "skeleton_native_calls", "skeleton_python_calls", "embedding_native_calls", "embedding_python_calls", "fallbacks", "skeleton_fallbacks"):
                value = getattr(record, key, None)
                if value is None:
                    continue
                if isinstance(value, (tuple, list)):
                    record_attrs[key] = record_attrs.get(key, 0) + len(value)
                elif isinstance(value, (int, float)):
                    record_attrs[key] = record_attrs.get(key, 0) + value
        row["backend_record_totals"] = record_attrs
    row["domain_outcomes"] = dict(sorted(outcomes.items()))
    row["refused"] = refused
    decal = bpy.data.objects.get(mesh_name + ".CFTUV_Decal")
    if decal is not None and "FINISHED" in status:
        me = decal.data
        row.update(verts=len(me.vertices), edges=len(me.edges), faces=len(me.polygons),
                   face_sizes={str(k): v for k, v in sorted(Counter(len(p.vertices) for p in me.polygons).items())})
        from cftuv.envelope_production_mesh import mesh_content_digest
        row["mesh_digest"] = mesh_content_digest(me)
        geometry = _geometry(me)
        # `cftuv_owner` - порядковый номер заявки владения: имя, а не геометрия. Его метки (`owner_labels_sha256`) отделены от самой геометрии и от РАЗБИЕНИЯ граней на
        # группы владения (`owner_partition_sha256`): переименование заявок двигает метки, а разбиение и геометрия остаются теми же.
        attributes = geometry["attributes"]
        owner = attributes.pop("cftuv_owner")
        domain = attributes["cftuv_domain"]
        blob = json.dumps(geometry, sort_keys=True).encode("utf-8")
        row["geometry_sha256"] = hashlib.sha256(blob).hexdigest()
        row["owner_labels_sha256"] = hashlib.sha256(json.dumps(owner).encode("utf-8")).hexdigest()
        groups: dict = {}
        for face, key in enumerate(zip(domain or (), owner or ())):
            groups.setdefault(key, []).append(face)
        row["owner_partition_sha256"] = hashlib.sha256(json.dumps(sorted(groups.values())).encode("utf-8")).hexdigest()
        geometry["owner_labels"] = owner
        blob = json.dumps(geometry, sort_keys=True).encode("utf-8")
        if _ARGS.geom_dir:
            directory = Path(_ARGS.geom_dir)
            directory.mkdir(parents=True, exist_ok=True)
            safe = spec.replace(":", "_").replace(".", "p")
            with gzip.open(directory / f"{safe}.json.gz", "wb") as handle:
                handle.write(blob)
    return row


def main():
    global _ARGS
    args = _arguments()
    _ARGS = args
    root = Path(args.root).resolve()
    _load_tree(root)
    _install_capture()
    results = []
    for spec in args.cases.split(","):
        try:
            row = _case(spec)
        except Exception:
            row = {"case": spec, "failure": traceback.format_exc()}
            print(row["failure"])
        results.append(row)
        print("FIELD_ROW", spec, row.get("verts"), row.get("faces"), (row.get("mesh_digest") or "")[:12], row.get("geometry_sha256", "")[:12],
              row.get("status"), row.get("button_seconds"), row.get("backend_ran"), flush=True)
    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps({"root": str(root), "settings": _SETTING_REPORT, "cases": results}, ensure_ascii=False, indent=1, sort_keys=True, default=str) + "\n", encoding="utf-8")
    try:
        from cftuv.envelope_domain_pool import shutdown_domain_pool
        shutdown_domain_pool()
    except Exception as exc:
        print("pool shutdown", exc)
    print("FIELD_DONE")


main()
