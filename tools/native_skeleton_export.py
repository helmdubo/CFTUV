"""Полевой корпус скелета: холодные подготовки мешей сцены кнопкой, каждый вызов `build_skeleton` — в запись.

    blender -b E:/testScene.blend --python-exit-code 1 --python tools/native_skeleton_export.py -- \\
        [--plan "building=0.2239;rounded_wall_noise_top=0.2239;sagging_wall=0.2239;2=0.2239;half_sphere=0.2239"] \\
        [--density 2] [--stretch 42] [--out <каталог корпуса скелета>] [--overwrite]

Меш считается кнопкой («Build Decal Mesh», `run_production`, последовательно: воркеров нет) на одной ширине, с ПУСТЫМ кэшем сессии: подготовка
каждого домена холодная (`prepare_for_production` обнуляет память разложений и счётчик неоплаченного перед `prepare_conveyor`, поэтому вызов
`build_skeleton` видит холодную память и свежий бюджет `PREPARE`, потраченный только тем, что успел мост). Подготовка не зависит от ширины, а ряд ширин
не добавляет новых подготовок: кэш сессии отдаёт готовую. Единица записи — `wavefront.conveyor.build_skeleton`: подменено ИМЯ, которое зовёт `_prepare_region`
(модуль импортировал его по имени), поэтому в запись попадает ровно то, что считала бы нативная вставка на том же месте.

Каждая подготовка (`wavefront.prepare_conveyor`) обёрнута: строка домена индекса несёт её чистое время (стенка минус время рекордера), секунды стадий
(`prepare.SKELETON`, `prepare.FACES`, `prepare.BRIDGE`, ...) и число записанных вызовов. Порядок вызовов `build_skeleton` внутри домена — порядок регионов (в v1 — один).

Корпус: `<корпус ядра>/skeleton/` (`--out` задаёт каталог целиком). Последняя строка при успехе: `NATIVE_SKELETON_EXPORT_OK <записей> <байт>`.
Сцена не сохраняется никогда; аддон берётся из дерева репозитория. Запускайте под питоном Blender (3.11): это питон продукта.
"""

from __future__ import annotations

import argparse
import shutil
import sys
import time
import traceback
from pathlib import Path

import bpy

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))
DEFAULT_PLAN = "building=0.2239;rounded_wall_noise_top=0.2239;sagging_wall=0.2239;2=0.2239;half_sphere=0.2239"


def _arguments():
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", default=str(ROOT))
    parser.add_argument("--plan", default=DEFAULT_PLAN)
    parser.add_argument("--density", type=int, default=2)
    parser.add_argument("--stretch", type=int, default=42)
    parser.add_argument("--out", default="")
    parser.add_argument("--preset", type=int, default=3)
    parser.add_argument("--max-bytes", type=int, default=300_000_000)
    parser.add_argument("--overwrite", action="store_true")
    return parser.parse_args(sys.argv[sys.argv.index("--") + 1 :] if "--" in sys.argv else [])


def _prepared(original, recorder, nc):
    """`prepare_conveyor`, который пишет строку домена подготовки: чистое время, секунды стадий, число записанных `build_skeleton`."""

    def prepare(*args, **kwargs):
        recorder.begin_domain(None, None, None)
        started = time.perf_counter()
        result = None
        try:
            result = original(*args, **kwargs)
        finally:
            wall = time.perf_counter() - started
            if result is not None:
                recorder.note_stages("prepare", result.timings)
                budget = result.work_budget
                recorder.context["domain_id"] = None if budget is None else budget.domain_id
            row = recorder.end_domain(wall, "RAISED" if result is None else str(result.outcome.value), 0.0)
            # Стадия SKELETON считала и время записи (пикл, сжатие): вычитается, как у операций полевого корпуса.
            if "prepare.SKELETON" in row["stages"]:
                row["stages"]["prepare.SKELETON"] -= row["op_overhead"][nc.OP_SKELETON]
            if result is not None:
                row["work_budget"] = nc.budget_state(result.work_budget)
        return result

    return prepare


def _install(ctx: dict, nc) -> None:
    """Подмена `conveyor.build_skeleton` и `wavefront.prepare_conveyor` рекордером."""

    import cftuv_envelope.wavefront as wavefront
    import cftuv_envelope.wavefront.conveyor as conveyor

    recorder = ctx["recorder"]
    conveyor.build_skeleton = recorder.wrap(nc.OP_SKELETON, nc.ORACLE[nc.OP_SKELETON])
    wavefront.prepare_conveyor = _prepared(wavefront.prepare_conveyor, recorder, nc)


def _prepare_directory(out: Path, overwrite: bool) -> None:
    if (out / "index.json").exists() or (out / "records").exists():
        if not overwrite:
            raise SystemExit(f"NATIVE_SKELETON_EXPORT_FAILED corpus directory {out} is not empty (pass --overwrite)")
        shutil.rmtree(out / "records", ignore_errors=True)
        (out / "index.json").unlink(missing_ok=True)


def _meshes(ctx: dict, ex, plan: dict) -> tuple:
    """Считает меши плана холодными подготовками; `(итоги мешей, отсутствующие меши)`."""

    summary: dict = {}
    missing = [name for name in plan if name not in bpy.data.objects]
    for name in plan:
        if name in missing:
            print(f"MESH_ABSENT {name}", flush=True)
            continue
        alphas = list(plan[name])
        print(f"{name}: {len(alphas)} widths", flush=True)
        opened = ex._open_mesh(ctx, name, fresh=True)
        rows_before, domains_before = len(ctx["recorder"].rows), len(ctx["recorder"].domains)
        row = {"mesh_digest": opened["revision"].digest, "seam_edges": len(opened["selected"]), "alphas": alphas}
        row.update(ex._run_alphas(ctx, name, opened, alphas))
        row["skeleton_records"] = len(ctx["recorder"].rows) - rows_before
        row["preparations"] = len(ctx["recorder"].domains) - domains_before
        summary[name] = row
    return summary, missing


def main() -> int:
    args = _arguments()
    import native_corpus_export as ex  # noqa: PLC0415 - модуль тянет bpy: только под Blender; ядро загружает `_context`

    ctx = ex._context(args)
    import native_corpus as nc
    import native_skeleton_corpus as sc

    out = Path(args.out) if args.out else sc.default_out("field")
    _prepare_directory(out, args.overwrite)
    description = nc.run_description(
        {"blender": bpy.app.version_string, "scene": bpy.data.filepath, "density": args.density, "stretch": args.stretch, "corpus": "skeleton_field"}
    )
    ctx["recorder"] = recorder = nc.Recorder(out, description, preset=args.preset, max_bytes=args.max_bytes, operations=nc.SKELETON_OPERATIONS)
    _install(ctx, nc)
    plan = ex.parse_plan(args.plan)
    failure = None
    summary, missing = {}, []
    try:
        summary, missing = _meshes(ctx, ex, plan)
    except Exception:  # noqa: BLE001 - причина идёт в отчёт и в код возврата
        failure = traceback.format_exc()
        print(failure)
    raised = [row for row in recorder.domains if row["outcome"] == "RAISED"]
    recorder.write_index({"meshes": summary, "missing_meshes": missing, "plan": plan, "failure": failure})
    from cftuv.envelope_domain_pool import shutdown_domain_pool

    shutdown_domain_pool()
    if failure is not None or missing or raised or not recorder.rows:
        print(f"NATIVE_SKELETON_EXPORT_FAILED failure={failure is not None} missing={missing} preparations_raised={len(raised)} records={len(recorder.rows)}")
        return 1
    print(f"NATIVE_SKELETON_EXPORT_OK {len(recorder.rows)} {recorder.bytes}")
    return 0


if __name__ == "__main__":
    code = main()
    if code:
        raise SystemExit(code)
