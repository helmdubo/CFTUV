"""Память стадии резки на настоящих мешах сцены: с памятью и без неё ответ каждого домена побайтно один.

    blender -b E:\\testscene.blend --python-exit-code 1 --python tools/blender_clip_memo_ab.py -- \\
        [--meshes building,rounded_wall_noise_top,sagging_wall] [--alphas 0.2239,0.5,0.55,0.987,1.3,4.5,5] \\
        [--density 2] [--stretch 42] [--out <json>]

Меш считается кнопкой («Build Decal Mesh», `run_production`, последовательно: воркеров нет) на каждой ширине списка; каждый
домен продуктового пути (`produce_domain`: покрытие и материализация, тот код, что у воркера пула) считается ДВАЖДЫ — без памяти
стадии резки (`clip_memo.memo_disabled`) и с ней — и сравнивается: исход, деталь, числа ответа (все счётчики, кроме
`EXACT_WORK_*`), диагностики, нормали, дайджесты и КАНОНИЧЕСКИЕ БАЙТЫ батча. Различие ответа — код возврата 1. Цена
(`EXACT_WORK_*`) перечисляется: она равна, когда память канонизации до резки та же, и записана ценой записи, когда нет.
Ширины в списке должны различаться (одна и та же ширина берётся из кэша сессии и ничего не считает), насыщенные пары
(`rounded_wall_noise_top` от 0.45, `sagging_wall` от 0.85, `building` от 4.5) дают попадания.

Последняя строка при успехе: CLIP_MEMO_AB_OK ... Аддон берётся из дерева репозитория (установленный снимается).
"""

from __future__ import annotations

import argparse
import json
import sys
import time
import traceback
from pathlib import Path

import bmesh
import bpy

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_MESHES = "building,rounded_wall_noise_top,sagging_wall"
DEFAULT_ALPHAS = "0.2239,0.5,0.55,0.987,1.3,4.5,5"
#: Поля результата домена, которые обязаны совпасть (ответ); `seconds`, `placement`, `clip_memo` и `labels` — не ответ.
ANSWER_FIELDS = (
    "outcome", "detail", "diagnostics", "content_digest", "offset_normals_digest", "vertex_normals",
    "offset_normal_law", "decal_topology_law", "normal", "source_normal", "chart_orientation",
)


def _arguments():
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", default=str(ROOT))
    parser.add_argument("--meshes", default=DEFAULT_MESHES)
    parser.add_argument("--alphas", default=DEFAULT_ALPHAS)
    parser.add_argument("--density", type=int, default=2)
    parser.add_argument("--stretch", type=int, default=42)
    parser.add_argument("--out", default="")
    return parser.parse_args(sys.argv[sys.argv.index("--") + 1 :] if "--" in sys.argv else [])


def _load_tree(root: Path) -> None:
    installed = sys.modules.get("cftuv")
    if installed is not None:
        try:
            installed.unregister()
        except Exception as exc:  # noqa: BLE001 - диагностика окружения, не причина отказа
            print("installed unregister failed:", type(exc).__name__, exc)
    for name in tuple(sys.modules):
        if name in {"cftuv", "cftuv_envelope"} or name.startswith(("cftuv.", "cftuv_envelope.")):
            del sys.modules[name]
    for path in (root / "kernel" / "src", root):
        text = str(path)
        if text in sys.path:
            sys.path.remove(text)
        sys.path.insert(0, text)
    import cftuv
    import cftuv_envelope

    assert Path(cftuv.__file__).resolve().parent == (root / "cftuv").resolve()
    assert Path(cftuv_envelope.__file__).resolve().parent == (root / "kernel" / "src" / "cftuv_envelope").resolve()
    cftuv.register()


def _select_seams(obj) -> list:
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
    chosen = []
    for edge in bm.edges:
        edge.select_set(bool(edge.seam))
        if edge.seam:
            chosen.append(edge.index)
    bmesh.update_edit_mesh(obj.data)
    return chosen


def _answer(result, canonical_json_bytes) -> tuple:
    counters = tuple(item for item in result.counters if not item[0].startswith("EXACT_WORK_"))
    batch = canonical_json_bytes(result.batch) if result.batch is not None else b""
    return tuple(getattr(result, name) for name in ANSWER_FIELDS) + (counters, batch)


def _compared_produce(original, stats, canonical_json_bytes, memo_disabled):
    """`produce_domain`, который считает домен без памяти и с ней, сверяет и отдаёт ответ С памятью."""

    def produce(patch_id, domain_id, prepared, alpha_text, **kwargs):
        with memo_disabled():
            off = original(patch_id, domain_id, prepared, alpha_text, **kwargs)
        on = original(patch_id, domain_id, prepared, alpha_text, **kwargs)
        stats["domains"] += 1
        stats["seconds_off"] += off.seconds
        stats["seconds_on"] += on.seconds
        stats[on.clip_memo or "NO_CLIP"] = stats.get(on.clip_memo or "NO_CLIP", 0) + 1
        if _answer(off, canonical_json_bytes) != _answer(on, canonical_json_bytes):
            stats["answer_differences"].append(f"patch {patch_id} alpha {alpha_text}: {on.clip_memo}")
        if off.counters != on.counters:
            stats["price_differences"] += 1
        return on

    return produce


def main() -> int:
    args = _arguments()
    root = Path(args.root).resolve()
    _load_tree(root)
    import cftuv.envelope_production_export as export_module
    from cftuv.analysis import build_analysis_bundle
    from cftuv.analysis_surface import source_revision_from_bmesh
    from cftuv.envelope_debug_session import WINDOW_MANAGER_SESSION_ATTRIBUTE, EnvelopeDebugSessionController
    from cftuv.envelope_request_policy import envelope_stretch_budget
    from cftuv_envelope.codec import canonical_json_bytes
    from cftuv_envelope.materialize.clip_memo import MEMO, memo_disabled

    stats: dict = {"domains": 0, "seconds_off": 0.0, "seconds_on": 0.0, "answer_differences": [], "price_differences": 0}
    export_module.produce_domain = _compared_produce(
        export_module.produce_domain, stats, canonical_json_bytes, memo_disabled
    )
    alphas = [float(item) for item in args.alphas.split(",")]
    assert len(set(alphas)) == len(alphas), "widths must differ: a repeated width is served from the session cache"
    report = {"root": str(root), "alphas": alphas, "meshes": {}}
    controller = getattr(bpy.context.window_manager, WINDOW_MANAGER_SESSION_ATTRIBUTE, None)
    if not isinstance(controller, EnvelopeDebugSessionController):
        controller = EnvelopeDebugSessionController()
        setattr(bpy.context.window_manager, WINDOW_MANAGER_SESSION_ATTRIBUTE, controller)
    failure = None
    try:
        for name in args.meshes.split(","):
            controller.clear()
            MEMO.clear()
            MEMO.reset_stats()
            before = {key: (value if not isinstance(value, list) else len(value)) for key, value in stats.items()}
            obj = bpy.data.objects[name]
            selected = _select_seams(obj)
            source_bm = bmesh.from_edit_mesh(obj.data)
            source_bm.faces.ensure_lookup_table()
            face_indices = tuple(face.index for face in source_bm.faces)
            revision = source_revision_from_bmesh(source_bm, obj, face_indices)
            key, data_key = int(obj.as_pointer()), int(obj.data.as_pointer())
            bundle = controller.get_analysis_bundle(
                key, data_key, revision, lambda: build_analysis_bundle(source_bm, face_indices, obj)
            )
            started = time.perf_counter()
            for alpha in alphas:
                export_module.run_production(
                    controller, bundle, frozenset(selected), alpha, source_object_key=key, source_data_key=data_key,
                    density=args.density, developable_stretch_budget=envelope_stretch_budget(args.stretch), workers=0,
                )
            row = {"seconds": round(time.perf_counter() - started, 2), **MEMO.snapshot()}
            row["domains_compared"] = stats["domains"] - before["domains"]
            report["meshes"][name] = row
            print(name, json.dumps(row), flush=True)
    except Exception:  # noqa: BLE001 - причина идёт в отчёт и в код возврата
        failure = traceback.format_exc()
        print(failure)
    stats["failure"] = failure
    report["stats"] = stats
    if args.out:
        Path(args.out).write_text(json.dumps(report, ensure_ascii=False, indent=1, sort_keys=True) + "\n", encoding="utf-8")
    from cftuv.envelope_domain_pool import shutdown_domain_pool

    shutdown_domain_pool()
    different = stats["answer_differences"]
    print(
        f"CLIP_MEMO_AB_{'OK' if not different and failure is None else 'FAILED'} domains={stats['domains']} "
        f"hits={stats.get('HIT', 0)} misses={stats.get('MISS', 0)} no_clip={stats.get('NO_CLIP', 0)} "
        f"answer_differences={len(different)} price_differences={stats['price_differences']} "
        f"seconds_without={stats['seconds_off']:.1f} seconds_with={stats['seconds_on']:.1f}"
    )
    for line in different[:10]:
        print("DIFFERENT", line)
    return 0 if not different and failure is None else 1


if __name__ == "__main__":
    code = main()
    if code:
        raise SystemExit(code)
