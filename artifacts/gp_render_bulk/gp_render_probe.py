"""Профиль и дамп GP_RENDER на настоящей кнопке: `building`, QUEUE, Fan Density 2.

Headless Blender, аддон берётся ИЗ РАБОЧЕГО ДЕРЕВА (не из установленной копии):

    blender --background --python-exit-code 1 --python gp_render_probe.py -- \
        --worktree <путь к рабочему дереву> --out <каталог> --tag <метка> \
        [--mesh building] [--density 2] [--replays 3] [--cprofile] [--scene E:\testscene.blend]

Что делает:
1. открывает сцену, выделяет ВСЕ швы меша (как `tools/blender_field_sweep.py`),
   ставит движок QUEUE и Fan Density, нажимает настоящую кнопку
   `hotspotuv.build_exact_reference_envelope_debug`, мерит настенное время;
2. печатает стадии профиля кнопки (GP_RENDER, GP_STROKES, GP_POINTS);
3. пишет детерминированный дамп GP-объекта `<out>/<tag>_dump.json`;
4. повторяет ТОЛЬКО отрисовку (`render_staged_envelope_debug`) на тех же
   аргументах `--replays` раз, мерит её отдельно, сверяет дамп каждого
   повтора с дампом кнопки и (при `--cprofile`) снимает cProfile одного
   повтора.
"""

from __future__ import annotations

import cProfile
import io
import json
import pstats
import sys
import time
from pathlib import Path

import bpy


def _arguments():
    values = sys.argv[sys.argv.index("--") + 1 :] if "--" in sys.argv else []
    parsed = {
        "worktree": None,
        "out": None,
        "tag": "run",
        "mesh": "building",
        "density": "2",
        "replays": 3,
        "cprofile": False,
        "instrument": False,
        "strings": False,
        "scene": r"E:\testscene.blend",
        "workers": None,
    }
    index = 0
    while index < len(values):
        key = values[index].lstrip("-")
        if key in {"cprofile", "instrument", "strings"}:
            parsed[key] = True
            index += 1
            continue
        value = values[index + 1]
        parsed[key] = int(value) if key in {"replays", "workers"} else value
        index += 2
    return parsed


def _profile_timings(source_name):
    text = bpy.data.texts.get(f"CFTUV_EnvelopeProfile_{source_name}.json")
    if text is None:
        return {}
    payload = json.loads(text.as_string())
    timings = {}
    for item in payload.get("timings", ()):
        if item.get("patch_domain_id") is None:
            timings[item["stage"]] = round(item["elapsed_seconds"], 4)
    counters = {
        item["name"]: item["value"]
        for item in payload.get("counters", ())
        if item["name"] in {"GP_STROKES", "GP_POINTS"}
        and item.get("patch_domain_id") is None
    }
    return {"timings": timings, "counters": counters}


_INSTRUMENTED = (
    "_lift_point",
    "_lift_plane_point",
    "_topology_lift",
    "_write_sidecar",
    "_staged_sidecar",
    "_write_gp_properties",
    "_print_profile",
    "_print_diagnostics",
    "clear_envelope_debug",
    "_render_topology_scene",
    "_render_exact_scene",
    "render_refused_domains",
    "render_queue_scene",
)


def _instrument(renderer):
    """Инклюзивные секунды и вызовы по функциям отрисовки (вложенность двойная)."""

    totals = {}

    def wrap(label, function):
        def timed(*call_args, **call_kwargs):
            begin = time.perf_counter()
            try:
                return function(*call_args, **call_kwargs)
            finally:
                entry = totals.setdefault(label, [0, 0.0])
                entry[0] += 1
                entry[1] += time.perf_counter() - begin

        return timed

    for name in _INSTRUMENTED:
        setattr(renderer, name, wrap(name, getattr(renderer, name)))
    writer_class = renderer.GreasePencilDebugWriter
    for name in ("__init__", "ensure_layer", "add_path", "commit", "stroke_count"):
        function = getattr(writer_class, name, None)
        if function is not None:
            setattr(writer_class, name, wrap("writer." + name, function))
    return totals


def _survey_expression_strings(captured):
    """Каких видов строки `x_expression`/`y_expression` несут точки сцен."""

    import re

    texts = []
    for scene in captured["args"][1]:
        for bucket in (scene.paths, scene.loops, scene.regions, scene.points):
            for record in bucket:
                exact = getattr(record, "exact_points", None)
                if exact is None and hasattr(record, "exact_point"):
                    exact = (record.exact_point,)
                if exact is None and hasattr(record, "outer_exact_points"):
                    exact = record.outer_exact_points
                for point in exact or ():
                    texts.append(point.x_expression)
                    texts.append(point.y_expression)
    kinds = {}
    for text in texts:
        kind = (
            "integer" if re.fullmatch(r"-?\d+", text)
            else "rational" if re.fullmatch(r"-?\d+/\d+", text)
            else "other"
        )
        kinds[kind] = kinds.get(kind, 0) + 1
    others = sorted({t for t in texts if not re.fullmatch(r"-?\d+(/\d+)?", t)})
    print(
        f"[PROBE] STRINGS total={len(texts)} unique={len(set(texts))} "
        f"kinds={kinds} unique_other={len(others)}"
    )
    for sample in others[:12]:
        print(f"[PROBE]   sample: {sample[:160]}")


def main():
    args = _arguments()
    worktree = Path(args["worktree"]).resolve()
    out = Path(args["out"])
    out.mkdir(parents=True, exist_ok=True)
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    import gp_dump
    from gp_probe_common import bootstrap, select_seams

    bootstrap(worktree)
    bpy.ops.wm.open_mainfile(filepath=args["scene"])
    obj = bpy.data.objects[args["mesh"]]
    selected = select_seams(obj)
    settings = bpy.context.scene.hotspotuv_settings
    settings.envelope_debug_engine = "QUEUE"
    settings.envelope_debug_fan_density = str(args["density"])
    if args["workers"] is not None:
        settings.envelope_debug_workers = args["workers"]
    print(
        f"[PROBE] version={bpy.app.version_string} mesh={obj.name} "
        f"seam_edges={selected} density={settings.envelope_debug_fan_density} "
        f"workers={settings.envelope_debug_workers}"
    )

    import cftuv.envelope_debug_renderer as renderer

    captured = {}
    original = renderer.render_staged_envelope_debug

    def recording(*call_args, **call_kwargs):
        captured["args"] = call_args
        captured["kwargs"] = call_kwargs
        return original(*call_args, **call_kwargs)

    renderer.render_staged_envelope_debug = recording

    started = time.perf_counter()
    outcome = bpy.ops.hotspotuv.build_exact_reference_envelope_debug()
    button_seconds = time.perf_counter() - started
    renderer.render_staged_envelope_debug = original
    assert outcome == {"FINISHED"}, outcome
    print(f"[PROBE] BUTTON_WALL_SECONDS={button_seconds:.3f}")

    gp_name = renderer.envelope_debug_object_name(obj)
    gp_obj = bpy.data.objects[gp_name]
    report = {
        "version": bpy.app.version_string,
        "button_wall_seconds": round(button_seconds, 3),
        "profile": _profile_timings(obj.name),
    }
    report["dump"] = gp_dump.write_dump(
        gp_obj, out / f"{args['tag']}_dump.json", obj.name
    )
    reference_sha = report["dump"]["sha256"]
    print(f"[PROBE] DUMP {json.dumps(report['dump']['sha256'])} "
          f"strokes={report['dump']['strokes']} points={report['dump']['points']}")

    replay_seconds = []
    profiled = False
    instrumented = _instrument(renderer) if args["instrument"] else None
    if args["strings"]:
        _survey_expression_strings(captured)
    for number in range(args["replays"]):
        renderer_args = captured["args"]
        renderer_kwargs = dict(captured["kwargs"])
        renderer_kwargs["profile"] = captured["kwargs"]["profile"]
        if args["cprofile"] and not profiled:
            profiler = cProfile.Profile()
            profiler.enable()
        begin = time.perf_counter()
        original(*renderer_args, **renderer_kwargs)
        replay_seconds.append(time.perf_counter() - begin)
        if args["cprofile"] and not profiled:
            profiler.disable()
            profiled = True
            stream = io.StringIO()
            stats = pstats.Stats(profiler, stream=stream)
            stats.sort_stats("cumulative").print_stats(45)
            (out / f"{args['tag']}_cprofile_cumulative.txt").write_text(
                stream.getvalue(), encoding="utf-8"
            )
            stream = io.StringIO()
            stats = pstats.Stats(profiler, stream=stream)
            stats.sort_stats("tottime").print_stats(30)
            (out / f"{args['tag']}_cprofile_tottime.txt").write_text(
                stream.getvalue(), encoding="utf-8"
            )
        if instrumented is not None:
            print("[PROBE] INSTRUMENT replay", number)
            for label, (calls, seconds) in sorted(instrumented.items()):
                print(f"[PROBE]   {label:<34} calls={calls:>7} seconds={seconds:8.4f}")
            report.setdefault("instrument", []).append(
                {k: [v[0], round(v[1], 4)] for k, v in instrumented.items()}
            )
            instrumented.clear()
        replay_dump = gp_dump.write_dump(
            bpy.data.objects[gp_name],
            out / f"{args['tag']}_replay{number}_dump.json",
            obj.name,
        )
        same = replay_dump["sha256"] == reference_sha
        print(
            f"[PROBE] REPLAY {number}: render={replay_seconds[-1]:.3f}s "
            f"dump_equals_button={same}"
        )
        if not same:
            report.setdefault("replay_mismatch", []).append(number)
    report["replay_render_seconds"] = [round(item, 4) for item in replay_seconds]
    (out / f"{args['tag']}_report.json").write_text(
        json.dumps(report, indent=2, ensure_ascii=False), encoding="utf-8"
    )
    print("[PROBE] REPORT " + json.dumps(report, ensure_ascii=False))
    print("GP_RENDER_PROBE_OK")


main()
