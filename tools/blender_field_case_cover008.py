"""Полевой случай `cover.008` (`buildings2_2.blend`): кнопка Build Decal Mesh на сохранённом выделении.

Случай — это константа `CASE`: файл (sha256), объект, сцена, выделение (рёбра, хранящиеся в меше; дайджест
отсортированных номеров), политика сцены (alpha, плотность, допуск растяжения, растворение UV, смещение, бэкенд)
и БАЗА отказов на 6a46ec0d (10 доменов из 1051). Скрипт ничего не сохраняет, меш источника не правит, UI не нужен.

Запуск (фоновый Blender БЕЗ `--factory-startup`; копия файла, не оригинал владельца):
  set PYTHONSAFEPATH=1
  blender -b <копия buildings2_2.blend> --python-exit-code 1 --python tools\\blender_field_case_cover008.py -- --out r.json [--workers 3] [--source installed] [--rows rows.json] [--geometry positions.json.gz]

`--rows`: запись ПО ДОМЕНАМ для судьи `artifacts/materialize_sweep/field_judge.py` (строка домена: исход, дайджест содержания батча, дайджест позиций, счётчики и диагностики материализатора;
номер случая `cover.008:<alpha>:<плотность>:<растяжение>:<патч>`, поэтому плотность судья читает из той же позиции, что у полевых случаев). `--geometry`: позиции вершин каждого построенного домена
(`{патч: {имя вершины: [x, y, z]}}`, gzip-JSON) для расстояний между двумя прогонами (этот же скрипт на основе и на дереве среза).

`--source worktree` (умолчание): пакеты берутся из этого дерева (установленный аддон из настроек снимается);
`--source installed`: что установлено и включено. Выход 0 — новых отказов нет; 1 — файл/выделение/политика не те
(`FIELD_CASE_*_MISMATCH`), число доменов иное, появился отказ вне базы либо вершин с `ADAPTER_WELD_MITER_FALLBACK`
стало больше базы, либо предполёт источника назвал не те домены не теми именами (ровно патч 18 `SOURCE_T_VERTEX` и патч 285
`SOURCE_FACE_SELF_INTERSECTION`), либо деталь отказа в консоли длиннее 240 знаков (`FIELD_CASE_REGRESSED`). Восстановленные домены базы
называются в выводе: это улучшение, а не сбой.

Повтор `SOURCE_SNAP_PLANE_PRESERVED_RETRY_V1` (`cftuv/envelope_snap_retry.py`): домены `CASE["snap_retry_domains"]` обязаны быть построены именно
повтором (диагностика исхода с именем первоначального отказа), и ни один другой домен повтор нести не вправе; расхождение - `FIELD_CASE_SNAP_RETRY_MISMATCH`.
"""

from __future__ import annotations

import gzip
import hashlib
import json
import os
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
CASE = {
    "blend_sha256": "5b5f2ea1c34ffec9eb6f10f46addc9d1ef35d907f18922dad7f26c064392231c",
    "object": "cover.008",
    "scene": "GEOMETRY",
    "selected_edges": 4317,
    "selection_sha256_16": "5cc5d0b5a0771d58",
    "policy": {"alpha": 0.04214220866560936, "density": "2", "max_stretch": 20, "dissolve_uv": 0.390625,
               "offset": 0.004999999888241291, "kernel_backend": "NATIVE"},
    "domains": 1051,
    # Отказы на 6a46ec0d (полевой отчёт владельца и воспроизведение): патч -> исход продуктового пути.
    "baseline_refused": {
        18: "ENVELOPE_DEBUG_PIPELINE_STAGE_FAILED", 285: "ENVELOPE_DEBUG_PIPELINE_STAGE_FAILED",
        40: "COVERAGE_IS_NOT_EXACT", 118: "COVERAGE_IS_NOT_EXACT", 312: "COVERAGE_IS_NOT_EXACT",
        629: "COVERAGE_IS_NOT_EXACT", 630: "COVERAGE_IS_NOT_EXACT", 1005: "COVERAGE_IS_NOT_EXACT",
        1002: "SURFACE_OFFSET_NORMAL_OPPOSES_TRIANGLE", 1007: "NO_GRID_SCALE_RESTORES_RELATIONS",
    },
    "baseline_weld_miter_fallbacks": 3,
    # Домены, которые строит повтор с масштабом решётки, сохраняющим плоскость патча (лотерея привязки на 6a46ec0d: отказы 118, 629, 630 -
    # DENSITY_RATIONAL_AUTHORITY_EXHAUSTED, 1005 - PLANAR_OWNER_INTERIOR_DIRECTION_REQUIRED, 1002 - SURFACE_OFFSET_NORMAL_OPPOSES_TRIANGLE).
    "snap_retry_domains": (118, 629, 630, 1002, 1005),
    # С COVER008-A предполёт источника называет два из десяти отказов ДО ядра (T-вершины патча 18, «бабочка» грани 1497 патча 285).
    # Ровно эти два домена и ровно этими именами; остальные восемь отказов базы прежние (из них 40, 312, 1007 остаются отказами и после повтора).
    "source_contact_refusals": {18: "SOURCE_T_VERTEX", 285: "SOURCE_FACE_SELF_INTERSECTION"},
    "console_detail_limit": 240,
}
RETRY_OUTCOME = "SOURCE_SNAP_PLANE_PRESERVED_RETRY_V1"


class FieldCaseError(RuntimeError):
    """Случай не тот, что записан: имя исхода в начале сообщения."""


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1 << 20), b""):
            digest.update(block)
    return digest.hexdigest()


def _use_packages(source: str) -> None:
    """Аддон из настроек Blender уже включён: для `worktree` его снимают и подставляют пакеты этого дерева."""

    import cftuv as present

    if source == "installed":
        return
    if Path(present.__file__).resolve().is_relative_to(ROOT):
        return
    present.unregister()
    for name in [n for n in sys.modules if n == "cftuv" or n.startswith("cftuv.")]:
        del sys.modules[name]
    for path in (str(ROOT / "kernel" / "src"), str(ROOT)):
        if path in sys.path:
            sys.path.remove(path)
        sys.path.insert(0, path)
    import cftuv

    cftuv.register()


def _enter_case(bpy, bmesh):
    """Объект, сцена и выделение случая; режим правки; всё сверено с `CASE`."""

    path = Path(bpy.data.filepath)
    if _sha256(path) != CASE["blend_sha256"]:
        raise FieldCaseError(f"FIELD_CASE_FILE_MISMATCH: {path} is not the frozen copy")
    obj = bpy.data.objects[CASE["object"]]
    scene = bpy.data.scenes[CASE["scene"]]
    bpy.context.window.scene = scene
    bpy.context.view_layer.objects.active = obj
    for other in bpy.context.selected_objects:
        other.select_set(False)
    obj.select_set(True)
    bpy.ops.object.mode_set(mode="EDIT")
    bpy.context.tool_settings.mesh_select_mode = (False, True, False)
    selected = [edge.index for edge in bmesh.from_edit_mesh(obj.data).edges if edge.select]
    digest = hashlib.sha256(json.dumps(selected).encode()).hexdigest()[:16]
    if len(selected) != CASE["selected_edges"] or digest != CASE["selection_sha256_16"]:
        raise FieldCaseError(f"FIELD_CASE_SELECTION_MISMATCH: {len(selected)} edges, digest {digest}")
    settings, mesh_settings = scene.hotspotuv_settings, scene.hotspotuv_decal_mesh
    seen = {"alpha": settings.envelope_debug_alpha, "density": settings.envelope_debug_fan_density,
            "max_stretch": settings.envelope_debug_max_stretch, "dissolve_uv": settings.envelope_debug_dissolve_uv_tolerance,
            "offset": mesh_settings.offset, "kernel_backend": mesh_settings.kernel_backend}
    for key, want in CASE["policy"].items():
        if (abs(seen[key] - want) > 1e-12) if isinstance(want, float) else (seen[key] != want):
            raise FieldCaseError(f"FIELD_CASE_POLICY_MISMATCH: {key} is {seen[key]!r}, the case records {want!r}")
    return obj, selected, settings, mesh_settings


def _run_button(bpy, bmesh, obj, selected, settings, mesh_settings, workers: int):
    """Те же вызовы, что у `HOTSPOTUV_OT_BuildEnvelopeDecalMesh.execute`, но с результатами в руках."""

    from cftuv.analysis import build_analysis_bundle
    from cftuv.analysis_surface import source_revision_from_bmesh
    from cftuv.envelope_kernel_backend import kernel_backend_of
    from cftuv.envelope_production_export import receipt_status_text, run_production
    from cftuv.envelope_production_mesh import build_mesh_arrays, write_decal_object
    from cftuv.envelope_production_operator import _runtime_key, _session
    from cftuv.envelope_request_policy import envelope_dissolve_uv_slide, envelope_stretch_budget

    bm = bmesh.from_edit_mesh(obj.data)
    bm.faces.ensure_lookup_table()
    face_indices = tuple(face.index for face in bm.faces)
    controller = _session(bpy.context)
    okey, dkey = _runtime_key(obj), _runtime_key(obj.data)
    revision = source_revision_from_bmesh(bm, obj, face_indices)
    bundle = controller.get_analysis_bundle(okey, dkey, revision, lambda: build_analysis_bundle(bm, face_indices, obj))
    started = time.perf_counter()
    run = run_production(
        controller, bundle, frozenset(selected), float(settings.envelope_debug_alpha),
        source_object_key=okey, source_data_key=dkey, density=settings.envelope_debug_fan_density,
        developable_stretch_budget=envelope_stretch_budget(settings.envelope_debug_max_stretch),
        silhouette_uv_slide=envelope_dissolve_uv_slide(settings.envelope_debug_dissolve_uv_tolerance),
        workers=workers, kernel_backend=kernel_backend_of(mesh_settings),
    )
    seconds = time.perf_counter() - started
    offset = float(mesh_settings.offset)
    receipt = write_decal_object(obj, run.results, offset=offset, material_name="CFTUV_Decal",
                                 width=float(settings.envelope_debug_alpha), arrays=build_mesh_arrays(run.results, offset))
    return run, receipt, receipt_status_text(receipt), seconds


def _verdict(run, receipt) -> dict:
    refused = {item.patch_id: item.outcome for item in run.results if not item.is_materialized}
    base = CASE["baseline_refused"]
    fallbacks = sum(int(detail.split()[0]) for _patch, name, detail in receipt.warnings if name == "ADAPTER_WELD_MITER_FALLBACK")
    retried = sorted(
        item.patch_id for item in run.results if any(line.startswith(RETRY_OUTCOME + ":") for line in item.diagnostics)
    )
    from cftuv.envelope_production_report import console_detail

    contact_names = set(CASE["source_contact_refusals"].values())
    return {
        "snap_retry": retried,
        "source_contact_refusals": {p: o for p, o in sorted(refused.items()) if o in contact_names},
        "longest_console_detail": max((len(console_detail(item.detail, item.outcome)) for item in run.results if not item.is_materialized), default=0),
        "domains": len(run.results), "refused": dict(sorted(refused.items())),
        "new_refusals": {p: o for p, o in sorted(refused.items()) if p not in base},
        "changed_outcome": {p: [base[p], o] for p, o in sorted(refused.items()) if p in base and base[p] != o},
        "recovered": sorted(p for p in base if p not in refused),
        "weld_miter_fallback_vertices": fallbacks,
    }


def _positions_of(item) -> dict:
    """Позиции вершин построенного домена: имя вершины (`semantic_location_ref`) -> [x, y, z]; у отказа пусто."""

    if not item.is_materialized:
        return {}
    return {
        vertex.semantic_location_ref.value: [float(vertex.position.x), float(vertex.position.y), float(vertex.position.z)]
        for vertex in item.batch.vertices
    }


def _domain_rows(run, positions: dict) -> list:
    """Строки для `field_judge.py`: одна на домен; цена (секунды) в строку не входит."""

    policy = CASE["policy"]
    prefix = f"{CASE['object']}:{policy['alpha']!r}:{policy['density']}:{policy['max_stretch']}"
    rows = []
    for item in sorted(run.results, key=lambda result: result.patch_id):
        placed = positions.get(item.patch_id, {})
        done = item.is_materialized
        rows.append({
            "case": f"{prefix}:{item.patch_id}",
            "operator": ["FINISHED"] if done else ["CANCELLED"],
            "status": item.outcome,
            "verts": len(placed) if done else None,
            "faces": len(item.batch.faces) if done else None,
            "mesh_digest": item.content_digest if done else None,
            "geometry_sha256": hashlib.sha256(json.dumps(placed, sort_keys=True).encode()).hexdigest() if done else None,
            "domain_outcomes": {item.outcome: 1},
            "refused": [] if done else [[item.patch_id, item.outcome, str(item.detail)[:160]]],
            "counters": dict(item.counters),
            "diagnostics": list(item.diagnostics),
        })
    return rows


def main() -> int:
    if os.environ.get("PYTHONSAFEPATH") != "1":
        print("PYTHONSAFEPATH_REQUIRED: kernel field runs require PYTHONSAFEPATH=1")
        return 1
    argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
    option = {argv[i]: argv[i + 1] for i in range(0, len(argv) - 1, 2)}
    import bmesh
    import bpy

    _use_packages(option.get("--source", "worktree"))
    try:
        obj, selected, settings, mesh_settings = _enter_case(bpy, bmesh)
    except FieldCaseError as exc:
        print(exc)
        return 1
    run, receipt, status, seconds = _run_button(bpy, bmesh, obj, selected, settings, mesh_settings, int(option.get("--workers", 3)))
    verdict = {**_verdict(run, receipt), "status": status, "run_seconds": round(seconds, 1)}
    if "--out" in option:
        Path(option["--out"]).write_text(json.dumps(verdict, indent=1, sort_keys=True), encoding="utf-8")
    if "--rows" in option or "--geometry" in option:
        positions = {item.patch_id: _positions_of(item) for item in run.results}
        if "--rows" in option:
            record = {"root": str(ROOT), "cases": _domain_rows(run, positions)}
            Path(option["--rows"]).write_text(json.dumps(record, indent=1, sort_keys=True), encoding="utf-8")
        if "--geometry" in option:
            with gzip.open(option["--geometry"], "wt", encoding="utf-8") as handle:
                json.dump({str(patch): placed for patch, placed in positions.items() if placed}, handle, sort_keys=True)
    print(status)
    print(f"recovered vs baseline: {verdict['recovered']}; new refusals: {verdict['new_refusals']}; outcome changes: {verdict['changed_outcome']}")
    more_weld = verdict["weld_miter_fallback_vertices"] > CASE["baseline_weld_miter_fallbacks"]
    contacts_differ = verdict["source_contact_refusals"] != CASE["source_contact_refusals"]
    console_too_long = verdict["longest_console_detail"] > CASE["console_detail_limit"]
    print(f"source contact refusals: {verdict['source_contact_refusals']} (expected {CASE['source_contact_refusals']}); longest console detail {verdict['longest_console_detail']}")
    regressed = verdict["domains"] != CASE["domains"] or bool(verdict["new_refusals"]) or more_weld or contacts_differ or console_too_long
    retry_mismatch = verdict["snap_retry"] != sorted(CASE["snap_retry_domains"])
    if retry_mismatch:
        print(f"FIELD_CASE_SNAP_RETRY_MISMATCH: built by the retry {verdict['snap_retry']}, the case records {sorted(CASE['snap_retry_domains'])}")
    print("FIELD_CASE_REGRESSED" if regressed else "FIELD_CASE_OK")
    return 1 if regressed or retry_mismatch else 0


if __name__ == "__main__":
    raise SystemExit(main())
