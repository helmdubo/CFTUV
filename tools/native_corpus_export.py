"""Выгрузка корпуса вызовов для нативного ускорителя ядра: настоящая кнопка на мешах сцены, каждый вызов двух операций — в запись.

    blender -b E:/testScene.blend --python-exit-code 1 --python tools/native_corpus_export.py -- \\
        [--plan "building=0.2239;rounded_wall_noise_top=0.2239,0.5;sagging_wall=0.987,0.2239;2=0.2239,0.5;half_sphere=0.2239,0.5"] \\
        [--series "rounded_wall_noise_top=0.2239;sagging_wall=0.987"] [--steps 0.01,0.001] [--kmax 5] \\
        [--extension-steps 0.02,0.05,0.1] [--density 2] [--stretch 42] [--out <каталог корпуса>] [--overwrite]

Меш считается кнопкой («Build Decal Mesh», `run_production`, последовательно: воркеров нет) на каждой ширине. Две операции ядра,
которые получат нативные реализации, подменены рекордером (`native_corpus.Recorder`): `wavefront.coverage._coverage_at` и
`materialize.clip.clip_geometry`; память стадии резки ВЫКЛЮЧЕНА, поэтому каждая резка считается и пишется. Каждый домен
(`produce_domain`) обёрнут: его чистое время (без времени рекордера) идёт в `index.json` вместе с долями покрытия и резки.
Ряд ширин (`--series`): `база * (1 ± шаг * k)`, k = 1..kmax, для каждого шага в обе стороны, округление до шести знаков;
ширины одного меша различны (одна и та же ширина берётся из кэша сессии и ничего не считает). Ряд обязан пересечь хотя бы одно
событие покрытия (число закрытых граней или вершин контура у какого-то региона домена меняется между соседними ширинами):
пока не пересёк, ряд расширяется шагами `--extension-steps`. Пересекающие пары пишутся в `index.json` (`series`).

Корпус: `<CFTUV_NATIVE_CORPUS или E:/cftuv_native_corpus>/<8 знаков HEAD>/` (`--out` задаёт каталог целиком). Последняя строка при
успехе: `NATIVE_CORPUS_EXPORT_OK <записей> <байт>`. Сцена не сохраняется никогда; аддон берётся из дерева репозитория.
"""

from __future__ import annotations

import argparse
import json
import os
import shutil
import sys
import time
import traceback
from pathlib import Path

import bmesh
import bpy

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))
DEFAULT_PLAN = (
    "building=0.2239;rounded_wall_noise_top=0.2239,0.5;sagging_wall=0.987,0.2239;2=0.2239,0.5;half_sphere=0.2239,0.5"
)
DEFAULT_SERIES = "rounded_wall_noise_top=0.2239;sagging_wall=0.987"
DEFAULT_STEPS = "0.01,0.001"
DEFAULT_EXTENSION_STEPS = "0.02,0.05,0.1"


def _arguments():
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", default=str(ROOT))
    parser.add_argument("--plan", default=DEFAULT_PLAN)
    parser.add_argument("--series", default=DEFAULT_SERIES)
    parser.add_argument("--steps", default=DEFAULT_STEPS)
    parser.add_argument("--kmax", type=int, default=5)
    parser.add_argument("--extension-steps", default=DEFAULT_EXTENSION_STEPS)
    parser.add_argument("--density", type=int, default=2)
    parser.add_argument("--stretch", type=int, default=42)
    parser.add_argument("--out", default="")
    parser.add_argument("--preset", type=int, default=3)
    parser.add_argument("--max-bytes", type=int, default=1_500_000_000)
    parser.add_argument("--overwrite", action="store_true")
    return parser.parse_args(sys.argv[sys.argv.index("--") + 1 :] if "--" in sys.argv else [])


def parse_plan(text: str) -> dict:
    """`имя=a,b;имя=a` -> `{имя: [a, b]}`: имя меша может быть любым (меш `2`), поэтому делитель — первое `=`."""

    plan: dict = {}
    for part in filter(None, (item.strip() for item in text.split(";"))):
        name, _, values = part.partition("=")
        plan[name.strip()] = [float(value) for value in values.split(",") if value.strip()]
    return plan


def series_alphas(base: float, steps, kmax: int) -> list:
    """`база * (1 ± шаг * k)`, k = 1..kmax, для каждого шага в обе стороны: округление до шести знаков, без повторов и без базы."""

    found: list = []
    for step in steps:
        for k in range(1, kmax + 1):
            for sign in (-1, 1):
                alpha = round(base * (1 + sign * step * k), 6)
                if alpha != base and alpha not in found:
                    found.append(alpha)
    return sorted(found)


def find_events(rows: list, mesh: str, alphas) -> list:
    """Пары соседних ширин РЯДА, между которыми у региона домена меняется топология: число граней или вершин (покрытие и резка).

    `alphas` — ширины ряда (база и её окрестность): соседство считается внутри них, а не среди всех ширин меша.
    """

    groups: dict = {}
    for row in rows:
        if row["mesh"] != mesh or row.get("faces") is None or row["domain_id"] is None or float(row["alpha"]) not in alphas:
            continue
        key = (row["domain_id"], row["op"], row["domain_call"])
        groups.setdefault(key, {})[float(row["alpha"])] = (row["faces"], row["vertices"], row["alpha"])
    events = []
    for (domain, op, region), by_alpha in sorted(groups.items(), key=lambda item: str(item[0])):
        ordered = [by_alpha[key] for key in sorted(by_alpha)]
        for left, right in zip(ordered, ordered[1:]):
            if left[:2] != right[:2]:
                events.append(
                    {"domain_id": domain, "op": op, "region": region, "from": left[2], "to": right[2],
                     "faces": [left[0], right[0]], "vertices": [left[1], right[1]]}
                )
    return events


def _compared_domain(original, recorder):
    """`produce_domain`, который считает домен как есть и пишет его чистое время и доли двух операций."""

    def produce(patch_id, domain_id, prepared, alpha_text, **kwargs):
        recorder.begin_domain(patch_id, domain_id, alpha_text)
        started = time.perf_counter()
        result = None
        try:
            result = original(patch_id, domain_id, prepared, alpha_text, **kwargs)
        finally:
            wall = time.perf_counter() - started
            if result is not None and str(result.outcome).endswith("RAISED"):
                print(f"  DOMAIN_RAISED {domain_id}: {result.detail}", flush=True)
            row = recorder.end_domain(
                wall, "RAISED" if result is None else str(result.outcome), 0.0 if result is None else float(result.seconds)
            )
            # Стадии, внутри которых звались операции, содержат время рекордера: вычитается, как из времени домена.
            for stage, op in (("coverage.COVERAGE_CLIP", "coverage_at"), ("materialize.CLIP", "clip_geometry")):
                if stage in row["stages"]:
                    row["stages"][stage] -= row["op_overhead"][op]
        return result

    return produce


def _staged(original, recorder, label: str):
    """Обёртка вызова ядра, чей результат несёт `timings`: стадии идут в строку домена под `метка.стадия`."""

    def staged(*args, **kwargs):
        result = original(*args, **kwargs)
        recorder.note_stages(label, getattr(result, "timings", ()))
        return result

    return staged


def _open_mesh(ctx: dict, name: str, *, fresh: bool) -> dict:
    """Меш в режиме правки со швами и пакет анализа сессии; `fresh` — начать с пустого кэша сессии (холодные подготовки)."""

    obj = bpy.data.objects[name]
    if fresh:
        ctx["controller"].clear()
    selected = ctx["ab"]._select_seams(obj)
    source_bm = bmesh.from_edit_mesh(obj.data)
    source_bm.faces.ensure_lookup_table()
    face_indices = tuple(face.index for face in source_bm.faces)
    revision = ctx["source_revision_from_bmesh"](source_bm, obj, face_indices)
    key, data_key = int(obj.as_pointer()), int(obj.data.as_pointer())
    bundle = ctx["controller"].get_analysis_bundle(
        key, data_key, revision, lambda: ctx["build_analysis_bundle"](source_bm, face_indices, obj)
    )
    ctx["recorder"].context.update(mesh=name, mesh_digest=revision.digest)
    return {"bundle": bundle, "key": key, "data_key": data_key, "selected": selected, "revision": revision}


def _run_alphas(ctx: dict, name: str, opened: dict, alphas: list) -> dict:
    """Ширины списка одна за другой кнопкой на одной подготовке сессии; `{домены, секунды}`."""

    recorder = ctx["recorder"]
    domains_before = len(recorder.domains)
    started = time.perf_counter()
    for alpha in alphas:
        began = time.perf_counter()
        count, domains = len(recorder.rows), len(recorder.domains)
        ctx["run_production"](
            ctx["controller"], opened["bundle"], frozenset(opened["selected"]), alpha,
            source_object_key=opened["key"], source_data_key=opened["data_key"], density=ctx["args"].density,
            developable_stretch_budget=ctx["stretch_budget"], workers=0,
        )
        print(f"  {name} alpha={alpha}: records +{len(recorder.rows) - count} domains +{len(recorder.domains) - domains} "
              f"{time.perf_counter() - began:.1f}s", flush=True)
    return {"domains": len(recorder.domains) - domains_before, "seconds": round(time.perf_counter() - started, 2)}


def _extend_series(ctx: dict, name: str, opened: dict, base: float, steps: list, row: dict) -> list:
    """Расширяет ряд меша шагами по очереди, пока не появится пара ширин, пересекающая событие покрытия."""

    events = find_events(ctx["recorder"].rows, name, set(row["series_alphas"]))
    for step in steps:
        if events:
            break
        extra = [alpha for alpha in series_alphas(base, [step], ctx["args"].kmax) if alpha not in row["alphas"]]
        print(f"  {name}: no coverage event crossed yet, extending the series with step {step}: {extra}", flush=True)
        part = _run_alphas(ctx, name, opened, extra)
        row["alphas"] += extra
        row["series_alphas"] += extra
        row["extended_with"] = [*row.get("extended_with", []), step]
        row["domains"] += part["domains"]
        row["seconds"] = round(row["seconds"] + part["seconds"], 2)
        events = find_events(ctx["recorder"].rows, name, set(row["series_alphas"]))
    return events


def _meshes(ctx: dict, plan: dict, series: dict, steps: list, extension: list) -> tuple:
    """Считает меши плана; `(итоги мешей, события рядов, отсутствующие меши)`."""

    summary: dict = {}
    events: dict = {}
    missing = [name for name in plan if name not in bpy.data.objects]
    for name in plan:
        if name in missing:
            print(f"MESH_ABSENT {name}", flush=True)
            continue
        alphas = list(plan[name])
        in_series: list = []
        if name in series:
            in_series = [series[name], *series_alphas(series[name], steps, ctx["args"].kmax)]
            alphas += [alpha for alpha in in_series if alpha not in alphas]
        assert len(set(alphas)) == len(alphas), "widths must differ: a repeated width is served from the session cache"
        print(f"{name}: {len(alphas)} widths", flush=True)
        opened = _open_mesh(ctx, name, fresh=True)
        row = {"mesh_digest": opened["revision"].digest, "seam_edges": len(opened["selected"]), "alphas": list(alphas),
               "series_alphas": in_series}
        row.update(_run_alphas(ctx, name, opened, alphas))
        summary[name] = row
        if name in series:
            events[name] = _extend_series(ctx, name, opened, series[name], extension, row)
    return summary, events, missing


def _context(args) -> dict:
    """Дерево репозитория загружено, сессия кнопки подготовлена. Модуль `native_corpus` импортируется ПОСЛЕ этого."""

    import blender_clip_memo_ab as ab

    ab._load_tree(Path(args.root).resolve())
    import cftuv.envelope_production_export as export_module
    from cftuv.analysis import build_analysis_bundle
    from cftuv.analysis_surface import source_revision_from_bmesh
    from cftuv.envelope_debug_session import WINDOW_MANAGER_SESSION_ATTRIBUTE, EnvelopeDebugSessionController
    from cftuv.envelope_request_policy import envelope_stretch_budget

    controller = getattr(bpy.context.window_manager, WINDOW_MANAGER_SESSION_ATTRIBUTE, None)
    if not isinstance(controller, EnvelopeDebugSessionController):
        controller = EnvelopeDebugSessionController()
        setattr(bpy.context.window_manager, WINDOW_MANAGER_SESSION_ATTRIBUTE, controller)
    return {
        "ab": ab, "args": args, "controller": controller, "export_module": export_module,
        "build_analysis_bundle": build_analysis_bundle, "source_revision_from_bmesh": source_revision_from_bmesh,
        "run_production": export_module.run_production, "stretch_budget": envelope_stretch_budget(args.stretch),
    }


def _install(ctx: dict, nc) -> None:
    """Подмена двух операций и `produce_domain` рекордером; память стадии резки выключена (каждая резка считается)."""

    os.environ["CFTUV_CLIP_MEMO"] = "0"
    nc.clip_memo.MEMO.enabled = False
    recorder = ctx["recorder"]
    nc.coverage._coverage_at = recorder.wrap(nc.OP_COVERAGE, nc.ORACLE[nc.OP_COVERAGE])
    nc.clip.clip_geometry = recorder.wrap(nc.OP_CLIP, nc.ORACLE[nc.OP_CLIP])
    export_module = ctx["export_module"]
    export_module.produce_domain = _compared_domain(export_module.produce_domain, recorder)
    import cftuv_envelope.materialize.domain as materialize_domain_module
    import cftuv_envelope.wavefront as wavefront

    # `produce_domain` берёт обе функции из модулей при каждом вызове, поэтому подмена видна ему.
    wavefront.conveyor_coverage = _staged(wavefront.conveyor_coverage, recorder, "coverage")
    materialize_domain_module.materialize_domain = _staged(materialize_domain_module.materialize_domain, recorder, "materialize")


def _prepare_directory(out: Path, overwrite: bool) -> None:
    if (out / "index.json").exists() or (out / "records").exists():
        if not overwrite:
            raise SystemExit(f"NATIVE_CORPUS_EXPORT_FAILED corpus directory {out} is not empty (pass --overwrite)")
        shutil.rmtree(out / "records", ignore_errors=True)
        (out / "index.json").unlink(missing_ok=True)


def main() -> int:
    args = _arguments()
    ctx = _context(args)
    import native_corpus as nc

    out = Path(args.out) if args.out else nc.corpus_directory(nc.git_head(ROOT))
    _prepare_directory(out, args.overwrite)
    description = nc.run_description(
        {"blender": bpy.app.version_string, "scene": bpy.data.filepath, "density": args.density, "stretch": args.stretch}
    )
    ctx["recorder"] = recorder = nc.Recorder(out, description, preset=args.preset, max_bytes=args.max_bytes)
    _install(ctx, nc)
    plan, series = parse_plan(args.plan), {name: values[0] for name, values in parse_plan(args.series).items()}
    steps = [float(item) for item in args.steps.split(",")]
    extension = [float(item) for item in args.extension_steps.split(",") if item]
    failure = None
    summary, events, missing = {}, {}, []
    try:
        summary, events, missing = _meshes(ctx, plan, series, steps, extension)
    except Exception:  # noqa: BLE001 - причина идёт в отчёт и в код возврата
        failure = traceback.format_exc()
        print(failure)
    unsettled = [name for name in series if name in summary and not events.get(name)]
    raised = [row for row in recorder.domains if row["outcome"].endswith("RAISED")]
    recorder.write_index({"meshes": summary, "missing_meshes": missing, "series": events, "plan": plan, "failure": failure})
    from cftuv.envelope_domain_pool import shutdown_domain_pool

    shutdown_domain_pool()
    for name, found in events.items():
        print(f"EVENTS {name}: {len(found)} crossed", json.dumps(found[:6]), flush=True)
    if failure is not None or unsettled or raised or not recorder.rows:
        print(f"NATIVE_CORPUS_EXPORT_FAILED failure={failure is not None} no_event={unsettled} "
              f"domains_raised={len(raised)} records={len(recorder.rows)}")
        return 1
    print(f"NATIVE_CORPUS_EXPORT_OK {len(recorder.rows)} {recorder.bytes}")
    return 0


if __name__ == "__main__":
    code = main()
    if code:
        raise SystemExit(code)
