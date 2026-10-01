"""Настоящая кнопка Envelope QUEUE на `building` в фоновом Blender: отпечаток и секунды.

Аддон берётся ИЗ ДЕРЕВА `--root` (а не из установленной копии): установленный
`cftuv` сперва снимается с регистрации, модули выгружаются, пути дерева и его
`kernel/src` встают первыми, и только потом дерево регистрируется. Так один и тот
же скрипт гоняет и ветку, и базовый коммит (`git worktree add` во временный
каталог) — отличие одно, `--root`.

    blender -b E:\\testscene.blend --python-exit-code 1 --python button_probe.py -- \\
        --root <дерево> --out <json> [--workers 8] [--mesh building] \\
        [--steps d2,d2,d1,alpha,d1]

Шаги: `dN` — кнопка при Fan Density N (первый — холодный, повтор — тёплый,
смена плотности — тёплые метрики при холодных подготовках), `alpha` либо
`alpha:<значение>` — ползунок alpha (лёгкий путь по тёплым подготовкам; по
умолчанию 0.375).

РАЗМЕЩЕНИЕ ОБЪЯВЛЕНО. Где домен считается (родитель либо воркер) — свойство
запуска, а не ответа, поэтому три вещи, которые это размещение называет, не
входят в отпечатки частей и печатаются рядом: счётчики `ENVELOPE_DOMAIN_POOL_*`,
стадия `QUEUE_POOL_WALL` и хвост панели `| pool wall N ms on W workers (...)`.
Всё остальное (sidecar, домены очереди, остальные счётчики, квитанции, свойства
и штрихи GP, строки панели, кэши сессии) сравнивается побитово. После КАЖДОГО шага пишется отпечаток
ответа: все артефакты, кроме секунд, свёрнуты в sha256, а сами части лежат рядом
(`parts`), чтобы расхождение называло, ЧТО разошлось.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import re
import sys
import time
import traceback
from pathlib import Path

import bmesh
import bpy


_MS = re.compile(r"\d+(?:\.\d+)? ?ms")
_SLOWEST = re.compile(r"slowest [0-9a-f]{3}")
# Хвост панели, который называет работу пула (`_pool_timing_suffix`).
_POOL_TEXT = re.compile(r" \| pool wall <ms> on \d+ workers \(times above are per-domain sums\)")
_POOL_COUNTER_PREFIX = "ENVELOPE_DOMAIN_POOL"
_POOL_STAGES = frozenset({"QUEUE_POOL_WALL"})
_SECONDS_KEYS = frozenset(
    {"prepare_seconds", "coverage_seconds", "contour_seconds", "timings"}
)


def _arguments() -> argparse.Namespace:
    tail = sys.argv[sys.argv.index("--") + 1 :] if "--" in sys.argv else []
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", required=True)
    parser.add_argument("--out", required=True)
    parser.add_argument("--workers", type=int, default=8)
    parser.add_argument("--mesh", default="building")
    parser.add_argument(
        "--steps",
        default="d2,d2,d2,d1,alpha:0.375,alpha:0.3,alpha:0.45,alpha:0.25,d1",
    )
    return parser.parse_args(tail)


def _load_tree(root: Path):
    """Снять установленный аддон и поднять дерево `root`."""

    installed = sys.modules.get("cftuv")
    if installed is not None:
        try:
            installed.unregister()
        except Exception as exc:  # noqa: BLE001 - диагностика окружения
            print("installed unregister failed:", type(exc).__name__, exc)
    for name in tuple(sys.modules):
        if name in {"cftuv", "cftuv_envelope"} or name.startswith(
            ("cftuv.", "cftuv_envelope.")
        ):
            del sys.modules[name]
    for path in (root / "kernel" / "src", root):
        text = str(path)
        if text in sys.path:
            sys.path.remove(text)
        sys.path.insert(0, text)
    import cftuv
    import cftuv_envelope

    assert Path(cftuv.__file__).resolve().parent == (root / "cftuv").resolve()
    assert (
        Path(cftuv_envelope.__file__).resolve().parent
        == (root / "kernel" / "src" / "cftuv_envelope").resolve()
    )
    cftuv.register()
    return cftuv


def _select_seams(obj) -> int:
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
    wanted = 0
    for edge in bm.edges:
        edge.select_set(bool(edge.seam))
        wanted += int(bool(edge.seam))
    bmesh.update_edit_mesh(obj.data)
    return wanted


def _sha(value) -> str:
    text = json.dumps(
        value, ensure_ascii=False, sort_keys=True, separators=(",", ":")
    )
    return hashlib.sha256(text.encode("utf-8")).hexdigest()[:16]


def _strip_seconds(value):
    if isinstance(value, dict):
        return {
            key: _strip_seconds(item)
            for key, item in value.items()
            if key not in _SECONDS_KEYS
        }
    if isinstance(value, list):
        return [_strip_seconds(item) for item in value]
    if isinstance(value, str):
        return _MS.sub("<ms>", value)
    return value


def _gp_digest(gp_obj):
    """Свёртка штрихов GP: слои, штрихи, точки; None, если API не отдаёт."""

    try:
        data = gp_obj.data
        rows = []
        for layer in data.layers:
            for frame in layer.frames:
                drawing = frame.drawing
                for stroke in drawing.strokes:
                    rows.append(
                        (
                            layer.name,
                            int(getattr(stroke, "material_index", 0)),
                            bool(getattr(stroke, "cyclic", False)),
                            [
                                (
                                    tuple(point.position),
                                    round(float(point.radius), 9),
                                )
                                for point in stroke.points
                            ],
                        )
                    )
        return {"strokes": len(rows), "digest": _sha(rows)}
    except Exception as exc:  # noqa: BLE001
        return {"unavailable": f"{type(exc).__name__}: {exc}"}


def _session(controller) -> dict:
    """Кэши сессии: ключи, счётчики сборок, отпечатки снапшотов."""

    from cftuv_envelope import codec

    snapshots = {}
    for key, value in sorted(controller._patch_metric_cache.items()):
        snapshot = getattr(value, "snapshot", None)
        if snapshot is None:
            snapshots[key[1][-8:]] = f"FAILURE:{value.outcome}:{_MS.sub('', value.message)[:80]}"
            continue
        snapshots[key[1][-8:]] = hashlib.sha256(codec.canonical_json_bytes(snapshot)).hexdigest()[:16]
    geometry = {
        key[1][-8:]: hashlib.sha256(codec.canonical_json_bytes(value.snapshot)).hexdigest()[:16]
        for key, value in sorted(controller._domain_geometry_cache.items())
    }
    return {
        "build_counts": controller.build_counts,
        # Ключ слоя ANALYSIS_BUNDLE несёт указатель объекта Blender: он меняется
        # от процесса к процессу и отпечатком быть не может.
        "cache_build_counts": _sha(
            sorted(
                (layer, "" if layer == "ANALYSIS_BUNDLE" else repr(key), value)
                for (layer, key), value in controller._cache_build_counts.items()
            )
        ),
        "patch_metric_snapshots": snapshots,
        "domain_geometry_snapshots": geometry,
        "preparation_keys": _sha(sorted(repr(key) for key in controller._conveyor_preparation_cache)),
        "preparation_count": len(controller._conveyor_preparation_cache),
        "queue_session": (
            None
            if controller.queue_session is None
            else {
                "entries": [
                    (patch_id, domain_id[-8:])
                    for patch_id, domain_id, _ in controller.queue_session.entries
                ],
                "receipts": _sha(
                    [
                        (item.patch_domain_id, item.stage.value, item.outcome, _MS.sub("<ms>", item.message))
                        for item in controller.queue_session.receipts
                    ]
                ),
                "density": controller.queue_session.density,
            }
        ),
    }


def _collect(obj, controller, settings, wall: float, label: str) -> dict:
    from cftuv.envelope_debug_renderer import (
        envelope_debug_object_name,
        envelope_debug_profile_text_name,
        envelope_debug_text_name,
    )

    sidecar = json.loads(bpy.data.texts[envelope_debug_text_name(obj)].as_string())
    profile = json.loads(bpy.data.texts[envelope_debug_profile_text_name(obj)].as_string())
    semantic = _strip_seconds(sidecar)
    queue_domains = {
        domain["patch_domain_id"][-8:]: _sha(_strip_seconds(domain))
        for domain in (sidecar.get("queue") or {}).get("domains", ())
    }
    counters = sorted(
        (item["name"], item["value"], (item["patch_domain_id"] or "")[-8:])
        for item in profile["counters"]
    )
    placement = [row for row in counters if row[0].startswith(_POOL_COUNTER_PREFIX)]
    counters = [row for row in counters if not row[0].startswith(_POOL_COUNTER_PREFIX)]
    receipts = [
        (
            item["patch_domain_id"][-8:],
            item["stage"],
            item["outcome"],
            _MS.sub("<ms>", item["message"]),
        )
        for item in profile["receipts"]
    ]
    stage_totals = {}
    stage_counts = {}
    for timing in profile["timings"]:
        stage_totals[timing["stage"]] = stage_totals.get(timing["stage"], 0.0) + timing["elapsed_seconds"]
        stage_counts[timing["stage"]] = stage_counts.get(timing["stage"], 0) + 1
    per_domain: dict = {}
    for timing in profile["timings"]:
        if timing["patch_domain_id"]:
            row = per_domain.setdefault(timing["patch_domain_id"][-8:], {})
            row[timing["stage"]] = row.get(timing["stage"], 0.0) + timing["elapsed_seconds"]
    heaviest = sorted(
        per_domain.items(),
        key=lambda item: -(item[1].get("QUEUE_PREPARE", 0.0) + item[1].get("QUEUE_COVERAGE", 0.0)),
    )[:8]
    gp_obj = bpy.data.objects.get(envelope_debug_object_name(obj.name))
    gp_props = (
        {key: _strip_seconds(gp_obj[key]) for key in sorted(gp_obj.keys()) if isinstance(gp_obj[key], str)}
        if gp_obj is not None
        else {}
    )
    settings_text = {
        # `slowest <домен>` — самый медленный по секундам, а секунды шумят.
        key: _POOL_TEXT.sub(
            "",
            _SLOWEST.sub("slowest <id>", _MS.sub("<ms>", str(getattr(settings, key, "")))),
        )
        for key in (
            "envelope_debug_status",
            "envelope_debug_outcome",
            "envelope_debug_domain_status",
            "envelope_debug_stage_summary",
            "envelope_debug_queue_timing",
        )
    }
    parts = {
        "sidecar": _sha(semantic),
        "queue_domains": _sha(queue_domains),
        "counters": _sha(counters),
        "receipts": _sha(receipts),
        "gp_props": _sha(gp_props),
        "gp_strokes": _sha(_gp_digest(gp_obj)) if gp_obj is not None else None,
        "settings": _sha(settings_text),
        "session": _sha(_session(controller)),
        "stage_names": _sha(sorted(set(stage_counts) - _POOL_STAGES)),
    }
    return {
        "label": label,
        "wall_seconds": round(wall, 3),
        "parts": parts,
        "gp": _gp_digest(gp_obj) if gp_obj is not None else None,
        "settings": settings_text,
        # Сырая строка таймингов панели: хвост пула виден как есть, не сравнивается.
        "queue_timing_raw": str(getattr(settings, "envelope_debug_queue_timing", "")),
        "stage_totals": {key: round(value, 3) for key, value in sorted(stage_totals.items())},
        "heaviest_domains": [
            {
                "domain": name,
                **{stage: round(value, 3) for stage, value in sorted(row.items()) if value > 0.01},
            }
            for name, row in heaviest
        ],
        "stage_counts": dict(sorted(stage_counts.items())),
        "domain_outcomes": _sha(
            [
                (d["patch_domain_id"][-8:], d["preparation_outcome"], d["coverage_outcome"])
                for d in (sidecar.get("queue") or {}).get("domains", ())
            ]
        ),
        "domains": len((sidecar.get("queue") or {}).get("domains", ())),
        "receipt_stages": sorted({item[1] for item in receipts}),
        "receipt_stage_counts": {
            stage: sum(1 for item in receipts if item[1] == stage)
            for stage in sorted({item[1] for item in receipts})
        },
        "pool_counters": {
            name: value for name, value, domain in placement if not domain
        },
        "pool_stages": sorted(set(stage_counts) & _POOL_STAGES),
        "detail": {
            "counters": counters,
            "receipts": receipts,
            "session": _session(controller),
            "queue_domain_digests": queue_domains,
        },
    }


def _step(label: str, obj, settings, controller) -> dict:
    if label.startswith("d"):
        settings.envelope_debug_fan_density = label[1:]
        _select_seams(obj)
        for name in (
            "CFTUV_EnvelopeDebug_" + obj.name + ".json",
            "CFTUV_EnvelopeProfile_" + obj.name + ".json",
        ):
            text = bpy.data.texts.get(name)
            if text is not None:
                bpy.data.texts.remove(text)
        started = time.perf_counter()
        outcome = bpy.ops.hotspotuv.build_exact_reference_envelope_debug()
        wall = time.perf_counter() - started
        assert outcome == {"FINISHED"}, outcome
        bpy.ops.object.mode_set(mode="OBJECT")
    elif label == "alpha" or label.startswith("alpha:"):
        value = float(label.split(":", 1)[1]) if ":" in label else 0.375
        started = time.perf_counter()
        settings.envelope_debug_alpha = value
        wall = time.perf_counter() - started
    else:
        raise ValueError(label)
    return _collect(obj, controller, settings, wall, label)


def main() -> None:
    args = _arguments()
    root = Path(args.root).resolve()
    _load_tree(root)
    obj = bpy.data.objects[args.mesh]
    settings = bpy.context.scene.hotspotuv_settings
    settings.envelope_debug_engine = "QUEUE"
    settings.envelope_debug_alpha = 0.5
    settings.envelope_debug_workers = args.workers
    from cftuv.envelope_debug_session import EnvelopeDebugSessionController

    controller = getattr(
        bpy.context.window_manager, "_cftuv_envelope_debug_session", None
    )
    if not isinstance(controller, EnvelopeDebugSessionController):
        controller = EnvelopeDebugSessionController()
        bpy.context.window_manager._cftuv_envelope_debug_session = controller
    controller.clear()
    result = {
        "root": str(root),
        "workers": args.workers,
        "mesh": args.mesh,
        "steps": [],
    }
    failure = None
    try:
        for label in args.steps.split(","):
            step = _step(label, obj, settings, controller)
            result["steps"].append(step)
            print(
                f"STEP {label}: wall {step['wall_seconds']} s, "
                f"domains {step['domains']}, stages {step['receipt_stage_counts']}"
            )
    except Exception:  # noqa: BLE001 - причина идёт в JSON
        failure = traceback.format_exc()
        print(failure)
    result["failure"] = failure
    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    Path(args.out).write_text(
        json.dumps(result, ensure_ascii=False, indent=1, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    from cftuv.envelope_domain_pool import shutdown_domain_pool

    shutdown_domain_pool()
    print("PROBE_DONE" if failure is None else "PROBE_FAILED")


main()
